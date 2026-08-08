# Cipher GIF Protocol v2

Cipher GIF is a client-side encrypted container rendered as an animated GIF.
Each frame keeps a carrier image, adds a visible `CIPHER GIF` inscription, and
stores encrypted bytes in a 2×2 color-variant matrix in the inscription band.

## Pipeline

1. Pack text or file bytes with encrypted filename and media type metadata.
2. Apply gzip only when it reduces the packed content by at least 64 bytes.
3. Derive an AES-256 key with PBKDF2-HMAC-SHA-256 and 600,000 iterations.
4. Encrypt once with AES-GCM, a random 16-byte salt, and a random 12-byte IV.
5. Split the authenticated envelope across 320 by 320 GIF frames.
6. Repeat each data frame twice and add a per-frame message id, index, total
   count, lengths, and CRC-32.

The frame CRC rejects accidental GIF damage. AES-GCM is the security boundary:
it authenticates the complete encrypted envelope and rejects a wrong password.

## Current limits

- Input content: 1 MiB.
- Transport: the original GIF is best. High-contrast 2×2 cells tolerate moderate
  GIF palette changes and resizing; conversion to video/WebP is unsupported.
- Redundancy: every distinct data frame repeats twice; single-chunk messages
  repeat to three animation frames.
- Passwords: at least eight characters. Random generated passwords are preferred.

Future versions can replace sequential chunks with Reed-Solomon or fountain
frames without changing the encrypted content container.
