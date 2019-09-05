// Copyright (C) Sebastian Baberowski.  See LICENSE.txt for details.

#pragma once

#include <stdint.h>

class Stream
{
public:
	virtual ~Stream() {}
	virtual size_t write(uint8_t byte) = 0;
    virtual size_t readBytes(char *buffer, size_t length) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t length) {
    	return readBytes((char *) buffer, length);
        }
};
