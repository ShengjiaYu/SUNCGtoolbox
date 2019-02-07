import os, pcl, numpy as np

base_dir = '/usr0/home/Datasets/SUNCG'
with open(os.path.join(base_dir, 'train_sceneId.txt')) as file:
    train_ids = file.read().splitlines()
with open(os.path.join(base_dir, 'test_sceneId.txt')) as file:
    test_ids = file.read().splitlines()
scene_ids = train_ids + test_ids

with open('train_sceneId.txt') as file:
    finished = file.read().splitlines()

num_partial_points = 0
num_complete_points = 0
unfinished = []
for scene_id in scene_ids:
    if scene_id in finished:
        scene_dir = os.path.join(base_dir, 'house', scene_id)
    # camera_file = os.path.join(scene_dir, 'cameras.txt')
    # if not os.path.exists(camera_file):
    #     unfinished.append(scene_id)
    #     continue
    # with open(camera_file) as file:
    #     num_cameras = len(file.readlines())
    # for i in reversed(range(num_cameras)):
    #     if not os.path.exists(os.path.join(scene_dir, 'imgs', '%06d_depth.png' % i)):
    #         unfinished.append(scene_id)
    #         break
        partial_path = os.path.join(scene_dir, 'house_partial.pcd')
        complete_path = os.path.join(scene_dir, 'house.pcd')
        try:
            partial = pcl.load(partial_path)
        except:
            continue
        try:
            complete = pcl.load(complete_path)
        except:
            continue
        num_partial_points += partial.size
        num_complete_points += complete.size
    else:
        unfinished.append(scene_id)
print('# scenes', len(finished))
print('Average # points in partial point cloud', num_partial_points / len(finished))
print('Average # points in complete point cloud', num_complete_points / len(finished))
with open('unfinished_sceneId.txt', 'w') as file:
    file.write('\n'.join(unfinished) + '\n')
