import os
from functools import partial
from multiprocessing.dummy import Pool
from subprocess import call
from termcolor import colored


base_dir = '/usr0/home/Datasets/SUNCG'
with open(os.path.join(base_dir, 'train_sceneId.txt')) as file:
    train_ids = file.read().splitlines()
with open(os.path.join(base_dir, 'test_sceneId.txt')) as file:
    test_ids = file.read().splitlines()
scene_ids = train_ids + test_ids

commands = []
sampling_binary = '/usr1/home/wyuan1/Repos/point-cvae/pcl_mod/build/bin/pcl_mesh_sampling'
leaf_size = 0.02
for scene_id in scene_ids:
    room_dir = os.path.join(base_dir, 'room', scene_id)
    room_names = [os.path.splitext(file)[0] for file in os.listdir(room_dir) if file.endswith('.obj')]
    room_paths = [os.path.join(room_dir, room_name) for room_name in room_names]
    commands += ['%s %s.obj %s.pcd -no_vis_result -leaf_size %f' % (sampling_binary, room, room, leaf_size)
                 for room in room_paths]

nproc = 16
pool = Pool(nproc)
for idx, return_code in enumerate(pool.imap(partial(call, shell=True), commands)):
    if return_code != 0:
        print(colored('Command \"%s\" failed' % commands[idx], 'yellow'))
    else:
        print(colored('-- Processed %d/%d' % (idx + 1, len(commands)), 'white', 'on_blue'))
