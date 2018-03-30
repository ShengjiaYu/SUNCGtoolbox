import os
from functools import partial
from multiprocessing.dummy import Pool
from subprocess import call
from termcolor import colored


house_dir = '/usr0/home/Datasets/SUNCG/house'
commands = []
for house_id in os.listdir(house_dir):
    house_path = os.path.join(house_dir, house_id)
    commands.append('./process_houses.sh %s' % house_path)

nproc = 16
pool = Pool(nproc)
for idx, return_code in enumerate(pool.imap(partial(call, shell=True), commands)):
    if return_code != 0:
        print(colored('Command \"%s\" failed' % commands[idx], 'yellow'))
    else:
        print(colored('-- Processed %d/%d' % (idx + 1, len(commands)), 'blue'))
