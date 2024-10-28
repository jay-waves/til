However, operational considerations can make TFC
      difficult to achieve. For example, if Alice sends a product idea
      to Bob in an email message, she wants data confidentiality for the
      message's content, and she might also want to conceal the
      destination of the message to hide Bob's identity from her
      competitors. However, the identity of the intended recipient, or
      at least a network address for that recipient, needs to be made
      available to the mail system. Thus, complex forwarding schemes may
      be needed to conceal the ultimate destination as the message
      travels through the open Internet (see: onion routing).

A [TFC](../../Security/ReadMe.md) service can be either full or partial:
      -  "Full TFC": This type of service conceals all traffic
         characteristics.
      -  "Partial TFC": This type of service either (a) conceals some
         but not all of the characteristics or (b) does not completely
         conceal some characteristic.

On point-to-point data links, full TFC can be provided by
      enciphering all PDUs and also generating a continuous, random data
      stream to seamlessly fill all gaps between PDUs. To a wiretapper,
      the link then appears to be carrying an unbroken stream of
      enciphered data. In other cases -- including on a shared or
      broadcast medium, or end-to-end in a network -- only partial TFC
      is possible, and that may require a combination of techniques. For
      example, a LAN that uses "carrier sense multiple access with
      collision detection" (CSMA/CD; a.k.a. "listen while talk") to
      control access to the medium, relies on detecting intervals of
      silence, which prevents using full TFC. Partial TFC can be
      provided on that LAN by measures such as adding spurious PDUs,
      padding PDUs to a constant size, or enciphering addresses just
      above the Physical Layer; but these measures reduce the efficiency
      with which the LAN can carry traffic. At higher protocol layers,
      SDUs can be protected, but addresses and other items of PCI must
      be visible at the layers below.