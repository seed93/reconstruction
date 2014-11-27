#pragma once
#include "rply.h"

class MyPlyIo
{
public:
	MyPlyIo(void(*getRgb)(float x, float y, float z, unsigned char rgb[]), char mode) :
		m_getRgb(getRgb), m_Mode(mode == 'a' ? PLY_ASCII : PLY_LITTLE_ENDIAN) {};
	int ReadAndWrite(char input[], char output[]);

private:
	void(*m_getRgb)(float x, float y, float z, unsigned char rgb[]);
	e_ply_storage_mode m_Mode;
};