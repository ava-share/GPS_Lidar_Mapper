''' Author: Dr. Talebpour
    Date: 8/25/2022 '''

with open('reprocessed_test1.pcd') as f:
    
    Lines = f.readlines()
    flag = True
    counter = 0
    file_counter = 0
    tmp = []
    for line in Lines:
        counter += 1
        if 11 < counter > 1920000: #start at 11 to skip header 
            counter = 0
            file_counter += 1
            filename = 'reprocessed_sub_pcd' + str(file_counter) + '.pcd'
            with open(filename, 'w') as f_w:
                f_w.write('# .PCD v0.7 - Point Cloud Data file format')
                f_w.write('\n')
                f_w.write('VERSION 0.7')
                f_w.write('\n')
                f_w.write('FIELDS x y z intensity')
                f_w.write('\n')
                f_w.write('SIZE 4 4 4 4')
                f_w.write('\n')
                f_w.write('TYPE F F F F')
                f_w.write('\n')
                f_w.write('COUNT 1 1 1 1')
                f_w.write('\n')
                f_w.write('WIDTH ' + str(len(tmp)))
                f_w.write('\n')
                f_w.write('HEIGHT 1')
                f_w.write('\n')
                f_w.write('VIEWPOINT 0 0 0 1 0 0 0')
                f_w.write('\n')
                f_w.write('POINTS ' + str(len(tmp)))
                f_w.write('\n')
                f_w.write('DATA ascii') 
                f_w.write('\n')
                for i in range(len(tmp)) :
                    f_w.write(tmp[i])
                #f_w.close()
		tmp = []
