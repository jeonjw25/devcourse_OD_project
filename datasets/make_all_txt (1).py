# -*- coding: utf-8 -*-

import os

path = "C:\\Users\\jeonj\\Desktop\\devcourse_OD_project-master\\datasets\\eval_data\\JPEGImages"
file_list = os.listdir(path)

print ("file_list: {}".format(file_list))

f = open("./all.txt", 'w', encoding='utf-8')

for file_name in file_list:
    ff = file_name.split('.')
    if len(ff) > 1 and ff[1] == 'png':
        f.write(ff[0]+'\n')

f.close()
