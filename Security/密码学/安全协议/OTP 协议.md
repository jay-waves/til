RFC4220, HMAC-based One Time Password.

HOTP = $\mathrm{HMAC}_{K}(Counter)$

常用验证码为 $HTOP \pmod{10^{6}}$

$T=\lfloor UnixTimeStamp/30 \rfloor$

TOTP = $HOTP_{K}(T)$.