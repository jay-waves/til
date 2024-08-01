## JSON (JavaScript Object Notation)

### Purpose

JSON (JavaScript Object Notation) is a lightweight data interchange format that is easy for humans to read and write and easy for machines to parse and generate. It is often used to transmit data between a server and a web application as an alternative to XML. JSON is language-independent.

JSON is widely used for data interchange (HTTP), configutration files (VSCode settings.json) and sotrage (MongoDB).

### Syntax

JSON is built on two structures:

1. Object:

**An unordered set of name/value pairs.** In JSON, an object is represented as a collection of key/value pairs enclosed in curly braces `{}`. Each key is a string enclosed in double quotes, followed by a colon, and then the value. Multiple key/value pairs are separated by commas.

   ```json
   {
     "name": "John Doe",
     "age": 30,
     "isStudent": false,
     "courses": ["Math", "Science"],
     "address": {
       "street": "123 Main St",
       "city": "Anytown"
     }
   }
   ```

2. Array:

**An ordered list of values.** An array is enclosed in square brackets `[]` and can contain objects, arrays, or primitive data types. Values are separated by commas.

   ```json
   [
     "apple",
     "banana",
     "cherry"
   ]
   ```

*Supported Data Types:*

| String  | "hello"       |
| ------- | ------------- |
| Number  | `42`, `3.14`  |
| Boolean | `true, false` |
| Array   | `[]`          |
| Object  | `{}`          |
| Null    |               |

### Examples

```json
{
  "id": 123,
  "name": "Alice",
  "email": "alice@example.com",
  "roles": ["user", "admin"]
}
```