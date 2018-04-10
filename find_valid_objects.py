import csv
import numpy as np
import os
import pcl


with open('metadata/ModelCategoryMapping.csv') as csv_file:
    with open('metadata/ValidObjects.csv', 'w') as output_file:
        reader = csv.DictReader(csv_file)
        writer = csv.writer(output_file)
        for row in reader:
            pcd_path = os.path.join('/usr0/home/Datasets/SUNCG/object', row['model_id'], '%s.pcd' % row['model_id'])
            if os.path.exists(pcd_path):
                pcd = np.array(pcl.load(pcd_path))
                dim = pcd.max(0) - pcd.min(0)
                if pcd.shape[0] < 800:
                    print(row['model_id'], row['fine_grained_class'], pcd.shape[0])
                elif dim[0] / dim[2] > 10 or dim[2] / dim[0] > 10:
                    print(row['model_id'], row['fine_grained_class'], dim[0] / dim[2], dim[2] / dim[0])
                else:
                    writer.writerow([row['index'], row['model_id'], row['fine_grained_class']])
