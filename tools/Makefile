# Makefile for log decoder utility

CC = gcc
CXX = g++
CFLAGS = -Wall -O2
CXXFLAGS = -Wall -O2 -std=c++11

# Paths
BUILD_DIR = ../build/sensing_fw
NANOPB_DIR = ../external/modules/lib/nanopb
INCLUDE_DIRS = -I$(BUILD_DIR) -I$(NANOPB_DIR) -I../include

# Source files
DECODER_SRC = log_decoder.cpp
NANOPB_SRCS = $(NANOPB_DIR)/pb_decode.c $(NANOPB_DIR)/pb_common.c
PROTO_SRCS = $(BUILD_DIR)/foot_sensor_messages.pb.c $(BUILD_DIR)/bhi360_sensor_messages.pb.c

# Target
TARGET = log_decoder

.PHONY: all clean

all: check_build $(TARGET)

check_build:
	@if [ ! -f $(BUILD_DIR)/foot_sensor_messages.pb.c ]; then \
		echo "Error: Protobuf files not found. Please build the main project first:"; \
		echo "  cd .. && west build"; \
		exit 1; \
	fi

$(TARGET): $(DECODER_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ $(DECODER_SRC) $(NANOPB_SRCS) $(PROTO_SRCS)
	@echo "Built $(TARGET) successfully"
	@echo "Usage: ./$(TARGET) <log_file_path>"

clean:
	rm -f $(TARGET)

# Helper target to decode the latest logs
decode-latest:
	@echo "This would decode the latest logs if we had access to the file system"
	@echo "Run on target: ./$(TARGET) /lfs1/hardware/foot_XXX.pb"
	@echo "Run on target: ./$(TARGET) /lfs1/hardware/bhi360_XXX.pb"