
## File I/O

```go
type Reader interface {
	Read(p []byte) (n int, err error)
}

type Writer interface {
	Write(p []byte) (n int, err error)
}
```

bufio:
```go
func lineByLine (file string) error {
	f, err := os.Open(file)
	if err != nil {
		return err
	}
	defer f.Close()
	r := bufio.NewReader(f)
	
	for {
		line, err := r.ReadString('\n') // read until \n
		if err == io.EOF {
			if len(line) != 0 {
				fmt.Println(line)
			}
			break
		}
		
		if err != nil {
			return err
		}
		
		fmt.Print(line)
	}
	return nil
}
```

## JSON

```go
import (
	"encoding/json"
	"fmt"
)

type UseAll struct {
	Name    string `json:"username"`
	Surname string `json:"surname"`
	Year    int    `json:"created"`
	Pass    string `josn:"-"`   // 序列化时, 会忽略该字段
}

func aaa() {
	useall := UseAll{Name: "Mike", Surname: "Tsoukalos", Year: 2023}
	t, err := json.Marshal(&useall)
	
	str := `{"username": "M.", "surname": "Ts", "created": 2024}`
	jsonRecord := []byte(str) // json.Unmarshal requires a byte slice
	temp := UseAll{}
	err = json.Unmarshal(josnRecord, &temp)
}

// 流式处理: 不要求 JSON 完整读入内存, 而是边读边解析. 
// json.Decoder 是对 io.Reader 的封装
func DeSerialize(e *json.Decoder, slice interface{}) error {
	return e.Decode(slice)
}

func Serialize(e *json.Encoder, slice interface{}) error {
	return e.Encode(slice)
}
```

## viper 

[viper](https://github.com/spf13/viper) 是一个命令行解析库, 比标准库 flag 强.

```go
package main 
import ( 
	"fmt" 
	"github.com/spf13/pflag" 
	"github.com/spf13/viper" 
)

// default: hardToGuess
// annotations: Password
pflag.StringP("password", "p", "hardToGuess", "Password")
...
pflag.Parse()

viper.BindPFlags(pflag.CommandLine)

name := viper.GetString("name")
password := viper.GetString("password")

// 环境变量
viper.BindEnv("GOMAXPROCS")
val := viper.Get("GOMAXPROCS")
if val != nil {
	...
}
```

## cobra 


