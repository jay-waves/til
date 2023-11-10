### `export`

Displays all environment variables. If you want to get details of a specific variable, use `echo $VARIABLE_NAME`.  

```bash
export
```

Example:

```bash
$ export
AWS_HOME=/Users/adnanadnan/.aws
LANG=en_US.UTF-8
LC_CTYPE=en_US.UTF-8
LESS=-R

$ echo $AWS_HOME
/Users/adnanadnan/.aws
```

这个命令还可以在当前 bash 进程修改变量 `export $var=hello`

### `man`

Shows the manual for specified command.  

```bash
man command
```

### `whatis`

whatis shows description for user commands, system calls, library functions, and others in manual pages

```bash
whatis something
```

Example:

```bash
$ whatis bash
bash (1)             - GNU Bourne-Again SHell
```

### `whereis`

whereis searches for executables, source files, and manual pages using a database built by system automatically.

```bash
whereis name
```

Example:

```bash
$ whereis php
/usr/bin/php
```

### `which`

which searches for executables in the directories specified by the environment variable PATH. This command will print the full path of the executable(s).

```bash
which program_name 
```

Example:

```bash
$ which php
/c/xampp/php/php
```

### `clear`

Clears content on window.


