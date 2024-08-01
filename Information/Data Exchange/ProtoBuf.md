## Protobuf (Protocol Buffers)

### Purpose
Protobuf, short for Protocol Buffers, is a language-agnostic, platform-neutral, extensible mechanism for serializing structured data. It was developed by Google to facilitate efficient communication between systems, particularly in scenarios where high performance and low latency are crucial. Protobuf is used to define data structures in a compact binary format, making it ideal for use in communication protocols, data storage, and configuration.

Protobuf is commonly used in remote procedure call (RPC) systems, data storage, and configuration files. It is particularly popular in microservices architectures, where efficient data serialization is essential for communication between services.

### Encoding Rules

- **Schema Definition**: Protobuf requires a schema definition (a .proto file) that describes the data structure, including the types of fields and their assigned field numbers. The schema is then compiled into language-specific classes, which are used to encode and decode the data.
- **Binary Encoding**: Data serialized with Protobuf is converted into a binary format, which is compact and efficient. This binary format includes both the field numbers and the corresponding values.
- **Field Types**: Protobuf supports various field types, including scalars (integers, floating points, booleans, etc.), strings, and composite types (messages, enumerations).
- **Backward and Forward Compatibility**: Protobuf supports schema evolution, allowing fields to be added or removed without breaking existing data.

### Examples

1. **Defining a Message in .proto File**:
   ```proto
   syntax = "proto3";

   message Person {
     int32 id = 1;
     string name = 2;
     string email = 3;
   }
   ```
   This defines a `Person` message with three fields: `id`, `name`, and `email`, each with a unique field number.

2. **Serialized Binary Data**:  
Given a `Person` object with `id = 123`, `name = "Alice"`, and `email = "alice@example.com"`, Protobuf will serialize this data into a compact binary format. This binary format is much smaller than an equivalent JSON or XML representation.
