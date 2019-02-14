from open3d import *
import numpy as np
import os, sys
import matplotlib.pyplot as plt


def sample_score(score_min, score_max, node, labels):
    candidates = np.where((labels[:, 0]>score_min) & (labels[:, 0]<=score_max)\
            & (instance==instance[node]))[0]
    return candidates


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
    label_fn = 'proposal_label.txt'

    with open(os.path.join(base_dir, train_fn)) as file:
        train_ids = file.read().splitlines()
    scene_ids = train_ids


    for ids in scene_ids:
        path = os.path.join(base_dir, 'house', ids, sub_dir)
        partial_pcd = read_point_cloud(os.path.join(path, 'house_partial.pcd'))
        partial_pcd.paint_uniform_color([0.5, 0.5, 0.5])
        partial_array = np.asarray(partial_pcd.points)

        partial_tree = KDTreeFlann(partial_pcd)

        instance = np.loadtxt(os.path.join(path, 'instance.txt'), dtype=int)
        labels = np.loadtxt(os.path.join(path, label_fn))

        obj_index = np.where(labels[:, 0]>0)[0]
        node = np.random.choice(obj_index, 1)
        node_points = np.where(instance==instance[node])[0]

        for score_min in np.arange(0.1, 0.9, 0.1):
            score_max = score_min + 0.1
            candidates = sample_score(score_min, score_max, node, labels)

            if len(candidates) == 0:
                print("no candidate")
                continue

            center_index = np.random.choice(candidates, 1)
            center = partial_pcd.points[center_index]
            radius = labels[center_index, 1]
            score = labels[center_index, 0]
            print(labels[center_index, :])

            [_, idx, _] = partial_tree.search_radius_vector_3d(center, radius)
            sphere = partial_array[idx]
            new_pcd = PointCloud()
            new_pcd.points = Vector3dVector(sphere)
            new_pcd.paint_uniform_color([0.5, 0.5, 0.5])

            for i, ori_i in enumerate(idx):
                if ori_i in node_points:
                    new_pcd.colors[i] = [1, 0, 0]
                if ori_i == center_index:
                    new_pcd.colors[i] = [0, 0, 1]

            show(new_pcd, os.path.join('/home/mengqing/Desktop', ids+'_'+str(instance[node][0])+'_'+str(score[0])+'_'+str(radius[0])))

