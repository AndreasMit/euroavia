C = gcc
CFLAGS = -Wall
SRCS = zeus_mission.c s_bus.c
OBJS = $(SRCS:.c=.o)
TARGET = zeus_mission

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
