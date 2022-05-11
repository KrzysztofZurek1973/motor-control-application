CC = gcc
CFLAGS = -g -Wall -Wextra -pthread
BUILD = build
TARGET = test-speed
FILES = $(wildcard *.c)
OBJS = $(patsubst %.c, $(BUILD)/%.o, $(FILES))
DEPS = $(wildcard *.h)
CLIBS = -L . -lpthread -lrt -lm

all: $(OBJS)
	@echo "\nStart compilation"
	@echo "--------------------------------------\n"
#	@echo $^
#	@echo $@
#	@echo $<
	$(CC) $^ $(CFLAGS) $(CLIBS) -o $(TARGET)
	@echo "\nDone"

#compile files in main directory
$(BUILD)/%.o: %.c $(DEPS)
	$(CC) -o $@ -c $< $(CFLAGS)

test:
	@echo FILES: $(FILES)
	@echo OBJS: $(OBJS)
	@echo DEPS: $(DEPS)

clean:
	@rm -f $(BUILD)/*.o
	@rm -f *.o
	@rm -f $(TARGET)
	@echo "Object files removed"
