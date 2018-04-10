import os
from functools import partial
from multiprocessing.dummy import Pool
from subprocess import call
from termcolor import colored


base_dir = '/usr0/home/Datasets/SUNCG/house'
scene_ids = [scene_id for scene_id in os.listdir(base_dir)]
scene_ids.sort()
scene_ids = scene_ids[:2200]
# with open('/usr0/home/Datasets/SUNCG/train_scenes.txt', 'w') as train_scenes:
#     train_scenes.write('\n'.join(scene_ids[:2000]))
# with open('/usr0/home/Datasets/SUNCG/valid_scenes.txt', 'w') as valid_scenes:
#     valid_scenes.write('\n'.join(scene_ids[2000:]))
commands = ['./process_houses.sh %s' % os.path.join(base_dir, scene_id) for scene_id in scene_ids]

nproc = 10
pool = Pool(nproc)
for idx, return_code in enumerate(pool.imap(partial(call, shell=True), commands)):
    if return_code != 0:
        print(colored('Command \"%s\" failed' % commands[idx], 'yellow'))
    else:
        print(colored('-- Processed %d/%d' % (idx + 1, len(commands)), 'white', 'on_blue'))
