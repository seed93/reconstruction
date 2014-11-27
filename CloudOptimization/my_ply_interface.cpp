#include "stdio.h"
#include "string.h"
#include "my_ply_interface.h"

struct MyPara
{
	p_ply oply;
	void(*getRgb)(float x, float y, float z, unsigned char rgb[]);
} myPara;

p_ply setPlyRead(char input[])
{
	p_ply iply = ply_open(input, NULL, 0, NULL);
	if (!iply) return 0;
	if (!ply_read_header(iply)) return 0;
	return iply;
}

p_ply setPlyWrite(char output[], e_ply_storage_mode mode, p_ply iply)
{
	p_ply oply = ply_create(output, mode, NULL, 0, NULL);
	p_ply_element element = NULL;

	if (!oply) return 0;
	if (!ply_add_obj_info(oply, "obj_info any data, in one line of free form text"))
		return 0;
	while ((element = ply_get_next_element(iply, element)))
	{
		long ninstances = 0;
		const char *element_name;
		ply_get_element_info(element, &element_name, &ninstances);
		if (!ply_add_element(oply, element_name, ninstances)) return 0;
		if (strcmp(element_name, "vertex") == 0)
		{
			if (!ply_add_property(oply, "x", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0))
				return 0;
			if (!ply_add_property(oply, "y", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0))
				return 0;
			if (!ply_add_property(oply, "z", PLY_FLOAT, (e_ply_type)0, (e_ply_type)0))
				return 0;
			if (!ply_add_property(oply, "red", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0))
				return 0;
			if (!ply_add_property(oply, "green", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0))
				return 0;
			if (!ply_add_property(oply, "blue", PLY_UCHAR, (e_ply_type)0, (e_ply_type)0))
				return 0;
		}
		else if (strcmp(element_name, "face") == 0)
		{
			if (!ply_add_property(oply, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT))
				return 0;
		}
	}
	if (!ply_write_header(oply))
		return 0;

	return oply;
}

static int ReadVertexCB(p_ply_argument argument)
{
	void *pdata;
	long value_index;
	static float point[3];
	static unsigned char rgb[3];

	ply_get_argument_user_data(argument, &pdata, &value_index);
	point[value_index] = (float)ply_get_argument_value(argument);
	if (value_index == 2)
	{
		((MyPara *)pdata)->getRgb(point[0], point[1], point[2], rgb);
		for (int i = 0; i < 3; i++)
			ply_write(((MyPara *)pdata)->oply, point[i]);
		for (int i = 0; i < 3; i++)
			ply_write(((MyPara *)pdata)->oply, rgb[i]);
	}

	return 1;
}

static int ReadFaceCB(p_ply_argument argument)
{
	void *pdata;
	ply_get_argument_user_data(argument, &pdata, NULL);
	ply_write(((MyPara *)pdata)->oply, ply_get_argument_value(argument));
	return 1;
}

int setup_callbacks(p_ply iply, p_ply oply, void(*getRgb)(float x, float y, float z, unsigned char rgb[]))
{
	myPara.oply = oply;
	myPara.getRgb = getRgb;
	ply_set_read_cb(iply, "vertex", "x", ReadVertexCB, &myPara, 0);
	ply_set_read_cb(iply, "vertex", "y", ReadVertexCB, &myPara, 1);
	ply_set_read_cb(iply, "vertex", "z", ReadVertexCB, &myPara, 2);
	ply_set_read_cb(iply, "face", "vertex_indices", ReadFaceCB, &myPara, 0);

	return 1;
}

int MyPlyIo::ReadAndWrite(char input[], char output[])
{
	p_ply iply, oply;
	iply = setPlyRead(input);
	if (!iply) return 1;
	oply = setPlyWrite(output, m_Mode, iply);
	if (!oply) return 1;
	if (!setup_callbacks(iply, oply, m_getRgb))
		return 1;
	if (!ply_read(iply)) return 1;
	if (!ply_close(iply)) return 1;
	if (!ply_close(oply)) return 1;
	return 0;
}