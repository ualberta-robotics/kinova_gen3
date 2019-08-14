// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016
// https://github.com/drewnoakes/joystick

#include "joystickBox.hh"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include "unistd.h"

JoystickBox::JoystickBox()
{
	openPath("/dev/input/js0");
}

JoystickBox::JoystickBox(int joystickNumber)
{
	std::stringstream sstm;
	sstm << "/dev/input/js" << joystickNumber;
	openPath(sstm.str());
}

JoystickBox::JoystickBox(std::string devicePath)
{
	openPath(devicePath);
}

JoystickBox::JoystickBox(std::string devicePath, bool blocking)
{
	openPath(devicePath, blocking);
}

void JoystickBox::openPath(std::string devicePath, bool blocking)
{
	// Open the device using either blocking or non-blocking
	_fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool JoystickBox::sample(JoystickBoxEvent* event)
{
	int bytes = read(_fd, event, sizeof(*event)); 

	if (bytes == -1)
		return false;

	// NOTE if this condition is not met, we're probably out of sync and this
	// Joystick instance is likely unusable
	return bytes == sizeof(*event);
}

bool JoystickBox::isFound()
{
	return _fd >= 0;
}

JoystickBox::~JoystickBox()
{
	close(_fd);
}

std::ostream& operator<<(std::ostream& os, const JoystickBoxEvent& e)
{
	os << "type=" << static_cast<int>(e.type)
		 << " number=" << static_cast<int>(e.number)
		 << " value=" << static_cast<int>(e.value);
	return os;
}


