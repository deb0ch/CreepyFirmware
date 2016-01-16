#
# Copyright (c) 2016 Thomas Chauvot de Beauchene
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

CXX			:= clang++

SRCDIR			:= src
OBJDIR			:= build

# These lines are needed to set immediate evaluation for
# these variables, instead of deferred evaluation which is unsuitable.
SRCS			:=
SUBDIRS			:=
CXXFLAGS		:=
LDFLAGS			:=

include $(SRCDIR)/module.mk

OBJS			:= $(addprefix $(OBJDIR)/, $(SRCS:.cpp=.o))

SRCS			:= $(addprefix $(SRCDIR)/, $(SRCS))

DEPS			:= $(OBJS:.o=.d)

TMPS			:= $(OBJS) $(OBJS:.o=.d)

CXXFLAGS		+= -W -Wall -Wextra -Werror
CXXFLAGS		+= -std=c++11
CXXFLAGS		+= -O3
CXXFLAGS		+= -MD
debug: CXXFLAGS		+= -g -g3 -ggdb
CXXFLAGS		+= -I./$(SRCDIR)

CXXFLAGS		+= -I./dxlservo/src
CXXFLAGS		+= -I./dxlservo/dxl_sdk_usb2ax/include

CXXFLAGS		+= $(shell pkg-config opencv --cflags)
CXXFLAGS		+= $(addprefix -I./$(SRCDIR)/, $(SUBDIRS))

LDFLAGS			+= -lboost_iostreams -lboost_system -lboost_filesystem
LDFLAGS			+= $(shell pkg-config opencv --libs)
LDFLAGS			+= -Ldxlservo -ldxlservo

debug: LDFLAGS		+= -g -g3 -ggdb

NAME			:= CreepyFirmware

all: $(NAME)

debug:	re

showflags:
	@printf "[\033[0;33mCompiler flags\033[0m] %s\n"
	@echo $(CXXFLAGS)
	@printf "[\033[0;33mLinker flags\033[0m] %s\n"
	@echo $(LDFLAGS)

dxlservo:
	@printf "[\033[0;34mBuilding library\033[0m] %s\n" "dxlservo"
	@make -C dxlservo

-include $(DEPS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@printf "[\033[0;32mCompiling\033[0m] %s\n" $<
	@$(COMPILE.c) $(OUTPUT_OPTION) $<

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@printf "[\033[0;32mCompiling\033[0m] %s\n" $<
	@$(COMPILE.cpp) $(OUTPUT_OPTION) $<

$(NAME): $(OBJS) dxlservo
	@printf "[\033[0;33mLinker flags\033[0m] %s\n"
	@echo $(LDFLAGS)
	@printf "[\033[0;34mLinking\033[0m] %s\n" $(NAME)
	@$(CXX) $(OBJS) -o $(NAME) $(LDFLAGS)
	@printf "[\033[0;35mDONE\033[0m]\n" $(NAME)

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	@printf "[\033[0;33mCompiler flags\033[0m] %s\n"
	@echo $(CXXFLAGS)
	@printf "[\033[0;35mGenerating folders...\033[0m] %s\n"
	@mkdir -p $(OBJDIR)
	@for dir in $(SUBDIRS);			\
	do					\
		mkdir -p $(OBJDIR)/$$dir;	\
	done

clean:
	@rm -rf $(TMPS)
	@printf "[\033[0;31mDeleted\033[0m] %s\n" $(OBJS)

fclean: clean
	@rm -rf $(NAME)
	@rm -rf $(OBJDIR)
	@printf "[\033[0;35mDeleted\033[0m] %s\n" $(NAME)
	@printf "[\033[0;35mDeleted\033[0m] %s\n" $(OBJDIR)

re:	fclean all

.PHONY:	all clean fclean re dxlservo
