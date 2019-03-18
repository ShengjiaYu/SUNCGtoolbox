from open3d import *
import numpy as np
import os, sys
import colorsys
import matplotlib.pyplot as plt
from scipy.spatial import distance

from functools import partial
from multiprocessing.dummy import Pool


base_dir = '/data/SUNCG'
sub_dir = 'new_sample'
train_fn = 'train.list'
output_fn = 'proposal_label_medoit.txt'

vis = False


def score2heatmap(score):
    np.clip(score, 0., 1.)
    h = (1-score) * 1.
    s = 1.
    l = score * 0.5
    r, g, b = colorsys.hls_to_rgb(h, l, s)
    return [r, g, b]


def show(pcd, fn='tmp'):
    vis = Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    image = vis.capture_screen_float_buffer()
    plt.imsave('{}.png'.format(fn), np.asarray(image))
    vis.destroy_window()


def process_obj(obj_pcd, obj_array):
    # compute distances
    distance_matrix = distance.cdist(obj_array, obj_array, 'euclidean')

    # compute medoit (sample 6 points as medoit)
    k = min(6, np.asarray(obj_pcd.points).shape[0]-1)
    idx = np.argpartition(distance_matrix.sum(axis=1), k)
    medoits = obj_array[idx[:k]]

    # compute radius
    r = distance_matrix[idx[:k]].max()

    # print(np.dot(trans[:3, :3], trans[:3, :3].T))
    # medoits = np.dot(trans[:3, :3], model['medoits'].T).T + trans[:3, 3].T
    return distance_matrix.max(axis=1), medoits, r


def create_label(ids):
    path = os.path.join(base_dir, 'house', ids, sub_dir)
    partial_pcd = read_point_cloud(os.path.join(path, 'house_partial.pcd'))
    num_points = np.asarray(partial_pcd.points).shape[0]

    with open(os.path.join(path, 'instance.txt')) as f:
        instance = [line.strip('\n') for line in f.readlines()]

    # transform all object point cloud
    object_pose_path = os.path.join(path, 'object_poses')
    pose_fns = os.listdir(object_pose_path)
    objs = {}
    for fn in pose_fns:
        # read object model point cloud
        node_id, model_id = os.path.splitext(fn)[0].split('_', 1)
        trans = np.loadtxt(os.path.join(object_pose_path, fn))
        #model = models[model_id]
        #obj_pcd = model['pointcloud']
        obj_pcd = read_point_cloud(
                    os.path.join(base_dir, 'object', model_id, model_id+'.pcd'))

        # transform to room frame
        obj_pcd.transform(trans)

        # down sample
        obj_pcd = voxel_down_sample(obj_pcd, voxel_size=0.1)
        obj_array = np.asarray(obj_pcd.points).astype(np.float16)
        # if num_points > 2000:
        #     k = int(num_points / 5000)
        #     obj_pcd = uniform_down_sample(obj_pcd, every_k_points=k)

        # process
        distances, medoits, r = process_obj(obj_pcd, obj_array)

        objs[node_id] = {'model_id': model_id,
                         'point_cloud': obj_pcd,
                         'kd_tree': KDTreeFlann(obj_pcd),
                         'distances': distances,
                         'medoits': medoits,
                         'r': r, # model['r'],
                         'num': obj_array.shape[0],
                         'max_score': 0,
                         'min_score': np.inf,
                         }

    if vis:
        partial_pcd.points.extend(Vector3dVector(medoits))
        partial_pcd.paint_uniform_color([0.5, 0.5, 0.5])
        np.asarray(partial_pcd.colors)[num_points:] = [0, 1, 0]

    partial_array = np.asarray(partial_pcd.points)

    # label each point in the partial scan
    scores = np.zeros(num_points)
    for i in range(num_points):
        node_id = instance[i]
        if node_id not in objs.keys():
            continue
        obj = objs[node_id]
        d = distance.cdist(obj['medoits'], partial_array[i][None, :]).min()
        alpha = 0.5
        score = (alpha * obj['r'] / (alpha * obj['r'] + d)) ** 2
        scores[i] = score
        if score > obj['max_score']:
            obj['max_score'] = score
        if score < obj['min_score']:
            obj['min_score'] = score


    with open(os.path.join(path, output_fn), 'w') as f:
        for i in range(num_points):
            node_id = instance[i]
            if node_id not in objs.keys():
                f.write('0 0\n')
                continue

            obj = objs[node_id]
            # print(obj['min_score'], obj['max_score'])
            if obj['max_score'] - obj['min_score'] == 0:
                score = 1.
            else:
                score = (scores[i] - obj['min_score']) / (obj['max_score'] - obj['min_score'])

            [_, idx, _] = obj['kd_tree'].search_knn_vector_3d(partial_array[i], 1)
            r = obj['distances'][idx]

            f.write('{} {}\n'.format(score, r))
            if vis:
                partial_pcd.colors[i] = score2heatmap(score)

    if vis:
        show(partial_pcd, os.path.join('/home/mengqing/Desktop', ids))


def pre_compute_models():
    # pre-compute model distance matrix
    models = {}
    for model_id in os.listdir(os.path.join(base_dir, 'object')):
        obj_pcd = read_point_cloud(
                    os.path.join(base_dir, 'object', model_id, model_id+'.pcd'))
        obj_pcd = voxel_down_sample(obj_pcd, voxel_size = 0.05)

        obj_array = np.asarray(obj_pcd.points)

        # compute distances
        distance_matrix = distance.cdist(obj_array, obj_array, 'euclidean')

        # compute medoit (sample 6 points as medoit)
        k = 6
        idx = np.argpartition(distance_matrix.sum(axis=1), k)
        medoits = obj_array[idx[:k]]

        # compute radius
        r = distance_matrix[idx[:k]].max()

        models[model_id] = {'point_cloud': obj_pcd,
                         'kd_tree': KDTreeFlann(obj_pcd),
                         'distance_matrix': distance_matrix,
                         'medoits': medoits,
                         'r': r,
                         'num': obj_array.shape[0],
                         }
    print("precomputing done!")
    return models


if __name__ == "__main__":

    with open(os.path.join(base_dir, train_fn)) as file:
        train_ids = file.read().splitlines()
    scene_ids = train_ids

    # process each scene
    if len(sys.argv) < 2:
        nproc = 4
        pool = Pool(nproc)
        log = open('log.txt', 'w')
        for i, result in enumerate(pool.imap(partial(create_label), scene_ids)):
            try:
                print(i, scene_ids[i])
            except:
                print(i, scene_ids[i], 'error occurs!')
                log.write('Error: {}'.format(scene_ids[i]))
                continue

    else:
        create_label(scene_ids[int(sys.argv[1])])

