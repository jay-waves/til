## XML

### Purpose

XML (eXtensible Markup Language) is a markup language designed for storing and transporting data in a human-readable and machine-readable format. It is both platform-independent and language-independent, making it an ideal choice for data exchange between different systems. XML is widely used for representing complex data structures, configuration files, and documents.

XML is widely used in web services (SOAP), configuration files, data interchange formats (e.g., RSS, Atom), document storage, and more.

### Rules

- *Elements*: The fundamental building blocks of XML, defined by tags. Each element has a start tag `<name>` and an end tag `</name>`, with the content in between.
- *Attributes*: Provide additional information about elements. They are defined within the start tag of an element, using a key-value pair syntax.
- *Hierarchical Structure*: XML documents have a tree-like structure, with nested elements representing parent-child relationships. **XML has only one root element**.
- *Prolog*: XML begins with a prolog that includes the XML declaration, specifying the XML version and character encoding, like: `<?xml version="1.0" encoding="UTF-8"?>`. After that comes DTD (document type definition) or XML Schema, like: `<!DOCTYPE note SYSTEM "book.dtd">`, which defining the data types and structures.

XML documents must be well-formed, adhering to syntax rules (e.g., properly nested tags). They can also be validated against a schema (DTD or XML Schema) to ensure they follow a specific structure.

Not well-formed XML/ Invalid XML:
- `<tag>` without `</tag>`
- `<a><b></a><b>` not nested
- `<tag attribute=value>` (corret: `<tag attribute="value">`)

Escape Characters:

|Character|Entity Name|Entity Number|Description|
|---|---|---|---|
|`&`|`&amp;`|`&#38;`|Ampersand|
|`<`|`&lt;`|`&#60;`|Less-than sign|
|`>`|`&gt;`|`&#62;`|Greater-than sign|
|`"`|`&quot;`|`&#34;`|Double quotation mark|
|`'`|`&apos;`|`&#39;`|Apostrophe (single quote)|
|`©`|`&copy;`|`&#169;`|Copyright symbol|
|`®`|`&reg;`|`&#174;`|Registered trademark symbol|
|`€`|`&euro;`|`&#8364;`|Euro sign|
|`¥`|`&yen;`|`&#165;`|Yen sign|
||`&nbsp;`|`&#160;`|Non-breaking space|

### Examples

1. **Simple XML Document**:
   ```xml
   <?xml version="1.0" encoding="UTF-8"?>
   <note>
     <to>Tove</to>
     <from>Jani</from>
     <heading>Reminder</heading>
     <body>Don't forget me this weekend!</body>
   </note>
   ```

2. **XML with Attributes**:
   ```xml
   <book isbn="978-3-16-148410-0">
     <title>XML Developer's Guide</title>
     <author>John Doe</author>
     <publisher>Example Press</publisher>
     <price>29.99</price>
   </book>
   ```
