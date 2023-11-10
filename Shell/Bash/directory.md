## 1.3. Directory Operations

| cd  | mkdir | pwd |
| --- | ----- | --- |
|  |       |     |

### a. `cd`

Moves you from one directory to other. Running this  

```bash
$ cd
```

moves you to home directory. This command accepts an optional `dirname`, which moves you to that directory.

```bash
cd dirname
```

### b. `mkdir`

Makes a new directory.  

```bash
mkdir dirname
```

You can use this to create multiple directories at once within your current directory.

```bash
mkdir 1stDirectory 2ndDirectory 3rdDirectory
```

You can also use this to create parent directories at the same time with the -p (or --parents) flag. For instance, if you wanted a directory named 'project1' in another subdirectory at '/samples/bash/projects/', you could run:

```bash
mkdir -p /samples/bash/projects/project1
mkdir --parents /samples/bash/projects/project1
```

Both commands above will do the same thing.
If any of these directories did no already exist, they would be created as well.

### c. `pwd`

Tells you which directory you currently are in.  

```bash
pwd
```