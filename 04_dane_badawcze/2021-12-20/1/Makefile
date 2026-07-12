
PROGNAME = sim-simp-rcv-L1

CC = gcc
LD = gcc


CFLAGS = -g -std=gnu99 -Wall
CFLAGS += -DNO_RCPP
CFLAGS += -I../lib
#CFLAGS += -I..

LDFLAGS = -lm 
#LDFLAGS += -lrt


#CFILES:=$(shell ls | grep .c)
#OBJS:=$(CFILES:%.c=%.o)

OBJDIR = build

OUT = $(OBJDIR)/$(PROGNAME)

#$(wildcard ../gnss_lib/*.c)
#$(wildcard ../gnss_lib/*.c)
# SRCS = $(wildcard *.c) $(wildcard ../*.c)  

SRCS = rcv_main.c 
SRCS += ../lib/my-math.c
SRCS += ../lib/SigGen.c 
SRCS += ../lib/TrackLoop.c
SRCS += ../lib/MeasLogicGps.c 
SRCS += ../lib/DecodeL1.c
SRCS += ../lib/FifoInt32.c
SRCS += ../lib/gps_l1_partity.c
SRCS += ../lib/bin-tools.c
SRCS += ../lib/BinMeas.c

#SRCS += ../lib/
#SRCS += ../lib/

OBJS=$(SRCS:.c=.o)

#OBJS=$(notdir $(SRCS:.c=.o))
#OBJ=$(join $(addsuffix ../obj/, $(dir $(SOURCE))), $(notdir $(SOURCE:.c=.o))) 


all: pre clean $(OUT)

$(OUT): pre $(OBJS)
	$(LD) $(LDFLAGS) -o $(OUT) $(addprefix $(OBJDIR)/, $(notdir $(OBJS)))
	@echo "\n--- BUILD OK"
	
%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $(OBJDIR)/$(notdir $@)

   
clean:
	rm -rf $(OBJDIR)/*
	@echo "CLEANED"
	
	@echo SRCS : $(SRCS)
	@echo OBJS : $(OBJS)
	@echo OBJS_DIRS : $(notdir $(OBJS))
	

pre:
	@mkdir -p $(OBJDIR)
    
    
run:
	@echo "-- RUNNING..."
	$(OUT)

memcheck:
	valgrind --tool=memcheck --leak-check=full $(OUT)

sgcheck:
	valgrind --tool=exp-sgcheck $(OUT)


.PHONY: run pre clean all

