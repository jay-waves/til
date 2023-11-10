现有一个名为 student.txt 的文件，每一行的内容用空格分隔。需要将该文件转化为 CSV 文件的格式。

**student.txt**

```
1 Mary Female
2 Jack Male
3 David Male
```

**（1）按行读取文本，存储成列表。**

```python
import pandas as pd

student_list = []
num_line = 3

fp = open('./student.txt', 'r') # 读取txt文件
for i in range(num_line):
    line_data = fp.readline().replace('\r','').replace('\n','')  # readline()读取一行文本；.replace('\r','').replace('\n','')去除行末的换行符
    line_list = [ str(i) for i in line_data.split(' ', 2)] # .split()按照空格划分文本
    student_list.append(line_list)
```

注：fp.readline()直接读取出的数据，每行行末会默认带上换行符 \\n，这里我们可以用 fp.readline().replace(’\\r’,’’).replace(’\\n’,’’) 去除换行符。

**（2）将列表数据存为CSV文件**

```python
print(student_list) # [['1', 'Mary', 'Female'], ['2', 'Jack', 'Male'], ['3', 'David', 'Male']]
col_index = ['Id', 'Name', 'Gender']
log_pd = pd.DataFrame(data=student_list, columns=col_index)
log_pd.to_csv('./student.csv', index=None)
fp.close()
```

最后得到的 student.csv 文件如下：  
![在这里插入图片描述](https://img-blog.csdnimg.cn/3bb6afafc2d04cd5b8ae39c7ae2aa5fd.png#pic_center)