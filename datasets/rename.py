import os

#file_path = 'C:\\Users\\jeonj\\Desktop\\devcourse_OD_project-master\\datasets\\train_data\\JPEGImages'
file_path = 'C:\\Users\\jeonj\\Desktop\\eval_add_txt'
file_names = os.listdir(file_path)

i = 195
for name in file_names:
    src = os.path.join(file_path, name)
    dst = 'eval_' + str(i) + '.txt'
    dst = os.path.join(file_path, dst)
    os.rename(src, dst)
    i += 1