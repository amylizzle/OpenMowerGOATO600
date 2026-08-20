## Reverse Engineering
Most of this firmware figuring was done by using a local instance of DeepSeek4-Flash-0731 to reverse engineer the protocols from the
hardware interface node binary on my GOAT O600. The python scripts contained here were generated to test this understanding.
The rest was a lot of experimentation and poking at things.

You can find an approximately accurate overview in findings.md, but I have not updated either the python scripts or findings document
as I've clarified misunderstandings and fixed errors. The authoratitive document here is the driver code, which hopefully has fewer 
mistakes. 