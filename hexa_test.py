hex_string = "08 00 02 07 25 07 D0 07 D0"
# Convert string to list of integers
data = [int(x, 16) for x in hex_string.split()]

# Calculate 8-bit sum checksum
checksum = sum(data) % 256

print(f"Checksum: {hex(checksum).upper()}")
print(f"Full Packet: {hex_string} {hex(checksum)[2:].upper()}")