from open3d import *
import numpy as np
import os
import colorsys
import matplotlib.pyplot as plt


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


if __name__ == "__main__":
    base_dir = '/data/SUNCG'
    sub_dir = 'new_sample_all_camera'
    train_fn = 'overlap_train_scenID_1_room_3.txt'
    output_fn = 'proposal_label.txt'

    with open(os.path.join(base_dir, train_fn)) as file:
        train_ids = file.read().splitlines()
    scene_ids = train_ids


    for ids in scene_ids:
        path = os.path.join(base_dir, 'house', ids, sub_dir)
        partial_pcd = read_point_cloud(os.path.join(path, 'house_partial.pcd'))
        partial_pcd.paint_uniform_color([0.5, 0.5, 0.5])
        partial_array = np.asarray(partial_pcd.points)

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
            obj_pcd = read_point_cloud(
                    os.path.join(base_dir, 'object', model_id, model_id+'.pcd'))

            # transform to room frame
            obj_pcd.transform(trans)
            obj_array = np.asarray(obj_pcd.points)

            # computer center
            center = obj_array.mean(axis=0)

            # compute radius
            xyz_min = obj_array.min(axis=0)
            xyz_max = obj_array.max(axis=0)
            radius = np.linalg.norm(xyz_max - xyz_min) / 2.


            objs[node_id] = {'model_id': model_id,
                             'point_cloud': obj_pcd,
                             'kd_tree': KDTreeFlann(obj_pcd),
                             'center': center,
                             'num': obj_array.shape[0],
                             'r': radius}
            #print(node_id, objs[node_id])

        # label each point in the partial scan
        with open(os.path.join(path, output_fn), 'w') as f:
            for i in range(partial_array.shape[0]):
                node_id = instance[i]
                if node_id not in objs.keys():
                    f.write('0 0\n')
                    continue

                # print(node_id)
                obj = objs[node_id]
                distance = np.linalg.norm(obj['center']-partial_array[i])
                # print(distance)
                alpha = 0.5
                score = (alpha * obj['r'] / (alpha * obj['r'] + distance)) ** 2

                #[_, idx, _] = obj['kd_tree'].search_knn_vector_3d(partial_array[i], obj['num'])
                #furthest_point = obj['point_cloud'].points[idx[-1]]
                #r = np.linalg.norm(furthest_point-partial_array[i])

                f.write('{} {}\n'.format(score, distance+obj['r']))
                partial_pcd.colors[i] = score2heatmap(score)
                #partial_pcd.colors[i] = [1, 0, 0]

        show(partial_pcd, os.path.join('/home/mengqing/Desktop', ids))

