## Text Operations

| awk  | cut | echo | egrep | fgrep | fmt  | |
| ---- | --- | ---- | ----- | ----- | ---- |--|
| grep | nl  | sed  | sort  | tr    | uniq | wc|

### a. `awk`

awk is the most useful command for handling text files. It operates on an entire file line by line. By default it uses whitespace to separate the fields. The most common syntax for awk command is

```bash
awk '/search_pattern/ { action_to_take_if_pattern_matches; }' file_to_parse
```

Lets take following file `/etc/passwd`. Here's the sample data that this file contains:

```
root:x:0:0:root:/root:/usr/bin/zsh
daemon:x:1:1:daemon:/usr/sbin:/usr/sbin/nologin
bin:x:2:2:bin:/bin:/usr/sbin/nologin
sys:x:3:3:sys:/dev:/usr/sbin/nologin
sync:x:4:65534:sync:/bin:/bin/sync
```

So now lets get only username from this file. Where `-F` specifies that on which base we are going to separate the fields. In our case it's `:`. `{ print $1 }` means print out the first matching field.

```bash
awk -F':' '{ print $1 }' /etc/passwd
```

After running the above command you will get following output.

```
root
daemon
bin
sys
sync
```

For more detail on how to use `awk`, check following [link](https://www.cyberciti.biz/faq/bash-scripting-using-awk).

### b. `cut`

Remove sections from each line of files

*example.txt*

```bash
red riding hood went to the park to play
```

*show me columns 2 , 7 , and 9 with a space as a separator*

```bash
cut -d " " -f2,7,9 example.txt
```

```bash
riding park play
```

### c. `echo`

Display a line of text

*display "Hello World"*

```bash
echo Hello World
```

```bash
Hello World
```

*display "Hello World" with newlines between words*

```bash
echo -ne "Hello\nWorld\n"
```

```bash
Hello
World
```

### d. `egrep`

Print lines matching a pattern - Extended Expression (alias for: 'grep -E')

*example.txt*

```bash
Lorem ipsum
dolor sit amet, 
consetetur
sadipscing elitr,
sed diam nonumy
eirmod tempor
invidunt ut labore
et dolore magna
aliquyam erat, sed
diam voluptua. At
vero eos et
accusam et justo
duo dolores et ea
rebum. Stet clita
kasd gubergren,
no sea takimata
sanctus est Lorem
ipsum dolor sit
amet.
```

*display lines that have either "Lorem" or "dolor" in them.*

```bash
egrep '(Lorem|dolor)' example.txt
or
grep -E '(Lorem|dolor)' example.txt
```

```bash
Lorem ipsum
dolor sit amet,
et dolore magna
duo dolores et ea
sanctus est Lorem
ipsum dolor sit
```

### e. `fgrep`

Print lines matching a pattern - FIXED pattern matching  (alias for: 'grep -F')

*example.txt*

```bash
Lorem ipsum
dolor sit amet,
consetetur
sadipscing elitr,
sed diam nonumy
eirmod tempor
foo (Lorem|dolor) 
invidunt ut labore
et dolore magna
aliquyam erat, sed
diam voluptua. At
vero eos et
accusam et justo
duo dolores et ea
rebum. Stet clita
kasd gubergren,
no sea takimata
sanctus est Lorem
ipsum dolor sit
amet.
```

*Find the exact string '(Lorem|dolor)' in example.txt*

```bash
fgrep '(Lorem|dolor)' example.txt
or
grep -F '(Lorem|dolor)' example.txt
```

```bash
foo (Lorem|dolor) 
```

### f. `fmt`

Simple optimal text formatter

*example: example.txt (1 line)*

```bash
Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.
```

*output the lines of example.txt to 20 character width*

```bash
cat example.txt | fmt -w 20
```

```bash
Lorem ipsum
dolor sit amet,
consetetur
sadipscing elitr,
sed diam nonumy
eirmod tempor
invidunt ut labore
et dolore magna
aliquyam erat, sed
diam voluptua. At
vero eos et
accusam et justo
duo dolores et ea
rebum. Stet clita
kasd gubergren,
no sea takimata
sanctus est Lorem
ipsum dolor sit
amet.
```

### g. `grep`

Looks for text inside files. You can use grep to search for lines of text that match one or many regular expressions, and outputs only the matching lines.  

```bash
grep pattern filename
```

Example:

```bash
$ grep admin /etc/passwd
_kadmin_admin:*:218:-2:Kerberos Admin Service:/var/empty:/usr/bin/false
_kadmin_changepw:*:219:-2:Kerberos Change Password Service:/var/empty:/usr/bin/false
_krb_kadmin:*:231:-2:Open Directory Kerberos Admin Service:/var/empty:/usr/bin/false
```

You can also force grep to ignore word case by using `-i` option. `-r` can be used to search all files under the specified directory, for example:

```bash
$ grep -r admin /etc/
```

And `-w` to search for words only. For more detail on `grep`, check following [link](https://www.cyberciti.biz/faq/grep-in-bash).

### h. `nl`

Number lines of files

*example.txt*

```bash
Lorem ipsum
dolor sit amet,
consetetur
sadipscing elitr,
sed diam nonumy
eirmod tempor
invidunt ut labore
et dolore magna
aliquyam erat, sed
diam voluptua. At
vero eos et
accusam et justo
duo dolores et ea
rebum. Stet clita
kasd gubergren,
no sea takimata
sanctus est Lorem
ipsum dolor sit
amet.
```

*show example.txt with line numbers*

```bash
nl -s". " example.txt 
```

```bash
     1. Lorem ipsum
     2. dolor sit amet,
     3. consetetur
     4. sadipscing elitr,
     5. sed diam nonumy
     6. eirmod tempor
     7. invidunt ut labore
     8. et dolore magna
     9. aliquyam erat, sed
    10. diam voluptua. At
    11. vero eos et
    12. accusam et justo
    13. duo dolores et ea
    14. rebum. Stet clita
    15. kasd gubergren,
    16. no sea takimata
    17. sanctus est Lorem
    18. ipsum dolor sit
    19. amet.
```

### i. `sed`

Stream editor for filtering and transforming text

*example.txt*

```bash
Hello This is a Test 1 2 3 4
```

*replace all spaces with hyphens*

```bash
sed 's/ /-/g' example.txt
```

```bash
Hello-This-is-a-Test-1-2-3-4
```

*replace all digits with "d"*

```bash
sed 's/[0-9]/d/g' example.txt
```

```bash
Hello This is a Test d d d d
```
 
 > [详见](https://www.cnblogs.com/liwei0526vip/p/5644163.html)

### j. `sort`

Sort lines of text files

*example.txt*

```bash
f
b
c
g
a
e
d
```

*sort example.txt*

```bash
sort example.txt
```

```bash
a
b
c
d
e
f
g
```

*randomize a sorted example.txt*

```bash
sort example.txt | sort -R
```

```bash
b
f
a
c
d
g
e
```

### k. `tr`

Translate or delete characters

*example.txt*

```bash
Hello World Foo Bar Baz!
```

*take all lower case letters and make them upper case*

```bash
cat example.txt | tr 'a-z' 'A-Z' 
```

```bash
HELLO WORLD FOO BAR BAZ!
```

*take all spaces and make them into newlines*

```bash
cat example.txt | tr ' ' '\n'
```

```bash
Hello
World
Foo
Bar
Baz!
```

### l. `uniq`

Report or omit repeated lines

*example.txt*

```bash
a
a
b
a
b
c
d
c
```

*show only unique lines of example.txt (first you need to sort it, otherwise it won't see the overlap)*

```bash
sort example.txt | uniq
```

```bash
a
b
c
d
```

*show the unique items for each line, and tell me how many instances it found*

```bash
sort example.txt | uniq -c
```

```bash
    3 a
    2 b
    2 c
    1 d
```

### m. `wc`

Tells you how many lines, words and characters there are in a file.  

```bash
wc filename
```

Example:

```bash
$ wc demo.txt
7459   15915  398400 demo.txt
```

Where `7459` is lines, `15915` is words and `398400` is characters.