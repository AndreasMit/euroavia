C = gcc
CFLAGS = -Wall
SRCS = ./src/zeus_mission.c ./src/s_bus.c ./src/telemetry.c ./src/mavlink_layer.c
OBJS = $(SRCS:.c=.o)
TARGET = ./build/zeus_mission

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) -I include/

clean:
	rm -f $(OBJS) $(TARGET)
