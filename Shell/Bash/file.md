## File Operations

| cat    | chmod | chown | cp    | diff | file | find |
| ------ | ----- | ----- | ----- | ---- | ---- | ---- |
| gunzip | gzip  | head  | lpq   | lpr  | lprm   | ls |
|  more   |  mv     | rm    | tail  | touch    |      ||

### a. `cat`

It can be used for the following purposes under UNIX or Linux.  

* Display text files on screen

* Copy text files  

* Combine text files  

* Create new text files  
  
  ```bash
  cat filename
  cat file1 file2 
  cat file1 file2 > newcombinedfile
  cat < file1 > file2 #copy file1 to file2

  cat > file # accept input from keyboard
  ```

### b. `chmod`

The chmod command stands for "change mode" and allows you to change the read, write, and execute permissions on your files and folders. For more information on this command check this [link](https://ss64.com/bash/chmod.html).

```bash
chmod -options filename
```

### c. `chown`

The chown command stands for "change owner", and allows you to change the owner of a given file or folder, which can be a user and a group. Basic usage is simple forward first comes the user (owner), and then the group, delimited by a colon.

```bash
chown -options user:group filename
```

### d. `cp`

Copies a file from one location to other.  

```bash
cp filename1 filename2
```

Where `filename1` is the source path to the file and `filename2` is the destination path to the file.

### e. `diff`

Compares files, and lists their differences.  

```bash
diff filename1 filename2
```

### f. `file`

Determine file type.  

```bash
file filename
```

Example:

```bash
$ file index.html
 index.html: HTML document, ASCII text
```

### g. `find`

Find files in directory

```bash
find directory options pattern
```

Example:

```bash
$ find . -name README.md
$ find /home/user1 -name '*.png'
```

### h. `gunzip`

Un-compresses files compressed by gzip.  

```bash
gunzip filename
```

### i. `gzcat`

Lets you look at gzipped file without actually having to gunzip it.  

```bash
gzcat filename
```

### j. `gzip`

Compresses files.  

```bash
gzip filename
```

### k. `head`

Outputs the first 10 lines of file  

```bash
head filename
```

### l. `lpq`

Check out the printer queue.  lpq=list printer queue

```bash
lpq
```

Example:

```bash
$ lpq
Rank    Owner   Job     File(s)                         Total Size
active  adnanad 59      demo                            399360 bytes
1st     adnanad 60      (stdin)                         0 bytes
```

### m. `lpr`

Print the file.  

```bash
lpr filename
```

### n. `lprm`

Remove something from the printer queue.  

```bash
lprm jobnumber
```

### o. `ls`

Lists your files. `ls` has many options: `-l` lists files in 'long format', which contains the exact size of the file, who owns the file, who has the right to look at it, and when it was last modified. `-a` lists all files, including hidden files. For more information on this command check this [link](https://ss64.com/bash/ls.html).  

```bash
ls option
```

Example:

<pre>
$ ls -la
rwxr-xr-x   33 adnan  staff    1122 Mar 27 18:44 .
drwxrwxrwx  60 adnan  staff    2040 Mar 21 15:06 ..
-rw-r--r--@  1 adnan  staff   14340 Mar 23 15:05 .DS_Store
-rw-r--r--   1 adnan  staff     157 Mar 25 18:08 .bumpversion.cfg
-rw-r--r--   1 adnan  staff    6515 Mar 25 18:08 .config.ini
-rw-r--r--   1 adnan  staff    5805 Mar 27 18:44 .config.override.ini
drwxr-xr-x  17 adnan  staff     578 Mar 27 23:36 .git
-rwxr-xr-x   1 adnan  staff    2702 Mar 25 18:08 .gitignore
</pre>

### p. `more`

Shows the first part of a file (move with space and type q to quit).  

```bash
more filename
```

### q. `mv`

Moves a file from one location to other.  

```bash
mv filename1 filename2
```

Where `filename1` is the source path to the file and `filename2` is the destination path to the file.

Also it can be used for rename a file.

```bash
mv old_name new_name
```

### r. `rm`

Removes a file. Using this command on a directory gives you an error.
`rm: directory: is a directory`
To remove a directory you have to pass `-r` which will remove the content of the directory recursively. Optionally you can use `-f` flag to force the deletion i.e. without any confirmations etc.

```bash
rm filename
```

### s. `tail`

Outputs the last 10 lines of file. Use `-f` to output appended data as the file grows.  

```bash
tail filename
```

### t. `touch`

Updates access and modification time stamps of your file. If it doesn't exists, it'll be created.

```bash
touch filename
```

Example:

```bash
$ touch trick.md
```