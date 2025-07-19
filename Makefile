# Makefile for Air Drums Project on QNX

# Compiler and flags
CC = qcc
CFLAGS = -Vgcc_ntoarmv7le -Wall -O2 -g
LDFLAGS = -Vgcc_ntoarmv7le -lm -lpthread -lasound

# Target and sources
TARGET = air_drums
SOURCES = air_drums.c
OBJECTS = $(SOURCES:.c=.o)

# Default target
all: $(TARGET)

# Build the main executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

# Compile source files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Install target (copy to appropriate location)
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/
	chmod 755 /usr/local/bin/$(TARGET)

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(TARGET)

# Create necessary directories and set permissions
setup:
	mkdir -p /var/log/air_drums
	mkdir -p /etc/air_drums
	chmod 755 /var/log/air_drums
	chmod 755 /etc/air_drums

# Debug build
debug: CFLAGS += -DDEBUG -g3
debug: $(TARGET)

# Help target
help:
	@echo "Available targets:"
	@echo "  all     - Build the air drums application"
	@echo "  install - Install the application to /usr/local/bin"
	@echo "  clean   - Remove build artifacts"
	@echo "  setup   - Create necessary directories"
	@echo "  debug   - Build with debug symbols"
	@echo "  help    - Show this help message"

.PHONY: all clean install setup debug help
