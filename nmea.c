
#include "nmea.h"

const char *nmea_delimiter[] = {

	"GGA", 	// 	Global Positioning System Fix Data
	"RMC", 	//	Recommended minimum specific GPS/Transit data
	"HDT", 	// 	Heading - Heading True
	"HDM",	// 	Heading - Heading Magnetic
	"DPT",  //	Depth
	"MTW",  //	Water temperature

};

#define		NMEA_GGA		nmea_delimiter[0]
#define		NMEA_RMC		nmea_delimiter[1]
#define		NMEA_HDT		nmea_delimiter[2]
#define		NMEA_HDM		nmea_delimiter[3]
#define		NMEA_DPT		nmea_delimiter[4]
#define		NMEA_MTW		nmea_delimiter[5]

float nmea_convert(float raw_degrees);
uint8_t nmea_checksum(const char *sentence);

//###########################################################################################################################
uint8_t nmea_checksum(const char *sentence)
{
	const char *n = sentence + 1;
  uint8_t chk = 0;
  while ('*' != *n && '\r' != *n)
	{
		if ('\0' == *n || n - sentence > 128)
      return 0;
    chk ^= (uint8_t) *n;
    n++;
  }
  return chk;
}
//###########################################################################################################################
inline float nmea_convert(float raw_degrees)
{
  int firstdigits = ((int)raw_degrees) / 100;
  float nexttwodigits = raw_degrees - (float)(firstdigits * 100.0f);
  return (float)(firstdigits + nexttwodigits / 60.0f);
}
//###########################################################################################################################
inline void nmea_callback(nmea_t *nmea)
{
	if (LL_USART_IsActiveFlag_RXNE(nmea->usart))
  {
    uint8_t tmp = LL_USART_ReceiveData8(nmea->usart);
		if (nmea->available)
			return;
    if (nmea->buf_index < nmea->buf_size - 1)
    {
      nmea->buf[nmea->buf_index++] = tmp;
      nmea->buf_time = HAL_GetTick();
    }
  }
  else
    nmea->usart->ICR = 0xFFFFFFFF;
}
//###########################################################################################################################
bool nmea_init(nmea_t *nmea, USART_TypeDef *usart, uint16_t buf_size)
{
	if (nmea->buf != NULL)
		return false;
	memset(nmea, 0, sizeof(nmea_t));
	nmea->usart = usart;
#if _NMEA_USE_FREERTOS != 0
	nmea->buf = (char*) pvPortMalloc(buf_size);
#else
	nmea->buf = (char*) malloc(buf_size);
#endif
	if (nmea->buf == NULL)
		return false;
	nmea->buf_size = buf_size;
	LL_USART_EnableIT_RXNE(nmea->usart);
	return true;
}
//###########################################################################################################################
void nmea_loop(nmea_t *nmea)
{
	while (nmea->lock)
		nmea_delay(1);
	nmea->lock = true;
	if ((nmea->buf_index > 0) && (HAL_GetTick() - nmea->buf_time >= 100))
	{
		char line[128];
		char *found = NULL;
		char *checksum = NULL;
		char checksum_read[2];
		bool end = false;

		//	+++	decode $xxGGA
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_GGA);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.time_h = ((str[0] - 48) * 10) + (str[1] - 48);
								nmea->gnss.time_m = ((str[2] - 48) * 10) + (str[3] - 48);
								nmea->gnss.time_s = ((str[4] - 48) * 10) + (str[5] - 48);
								nmea->gnss.valid.time = 1;
							break;
							case 3:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.latitude_tmp = (float)atof(str);
							break;
							case 4:
								if (strcmp(str, "S") == 0)
								{
									nmea->gnss.latitude_deg = -nmea_convert(nmea->gnss.latitude_tmp);
									nmea->gnss.valid.latitude = 1;
								}
								else if (strcmp(str, "N") == 0)
								{
									nmea->gnss.latitude_deg = nmea_convert(nmea->gnss.latitude_tmp);
									nmea->gnss.valid.latitude = 1;
								}
							break;
							case 5:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.longitude_tmp = (float)atof(str);
							break;
							case 6:
								if (strcmp(str, "W") == 0)
								{
									nmea->gnss.longitude_deg = -nmea_convert(nmea->gnss.longitude_tmp);
									nmea->gnss.valid.longitude = 1;
								}
								else if (strcmp(str, "E") == 0)
								{
									nmea->gnss.longitude_deg = nmea_convert(nmea->gnss.longitude_tmp);
									nmea->gnss.valid.longitude = 1;
								}
							break;
							case 7:

							break;
							case 8:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.satellite = atoi(str);
								nmea->gnss.valid.satellite = 1;
							break;
							case 9:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.precision_m = (float)atof(str);
								nmea->gnss.valid.precision = 1;
							break;
							case 10:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.altitude_m = (float)atof(str);
								nmea->gnss.valid.altitude = 1;
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxGGA

		//	+++	decode $xxRMC
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_RMC);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.time_h = ((str[0] - 48) * 10) + (str[1] - 48);
								nmea->gnss.time_m = ((str[2] - 48) * 10) + (str[3] - 48);
								nmea->gnss.time_s = ((str[4] - 48) * 10) + (str[5] - 48);
								nmea->gnss.valid.time = 1;
							break;
							case 3:

							break;
							case 4:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.latitude_tmp = (float)atof(str);
							break;
							case 5:
									if (strcmp(str, "S") == 0)
									{
										nmea->gnss.latitude_deg = -nmea_convert(nmea->gnss.latitude_tmp);
										nmea->gnss.valid.latitude = 1;
									}
								else if (strcmp(str, "N") == 0)
								{
									nmea->gnss.latitude_deg = nmea_convert(nmea->gnss.latitude_tmp);
									nmea->gnss.valid.latitude = 1;
								}
							break;
							case 6:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.longitude_tmp = (float)atof(str);
							break;
							case 7:
								if (strcmp(str, "W") == 0)
								{
									nmea->gnss.longitude_deg = -nmea_convert(nmea->gnss.longitude_tmp);
									nmea->gnss.valid.longitude = 1;
								}
								else if (strcmp(str, "E") == 0)
								{
									nmea->gnss.longitude_deg = nmea_convert(nmea->gnss.longitude_tmp);
									nmea->gnss.valid.longitude = 1;
								}
							break;
							case 8:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.speed_knots = (float)atof(str);
								nmea->gnss.valid.speed_knots = 1;
							break;
							case 9:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.course_deg = (float)atof(str);
								nmea->gnss.valid.course = 1;
							break;
							case 10:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->gnss.date_d = ((str[0] - 48) * 10) + (str[1] - 48);
								nmea->gnss.date_m = ((str[2] - 48) * 10) + (str[3] - 48);
								nmea->gnss.date_y = ((str[4] - 48) * 10) + (str[5] - 48);
								nmea->gnss.valid.date = 1;
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxRMC

		//	+++	decode &xxHDT
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_HDT);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->compass.true_course_deg = (float)atof(str);
								nmea->compass.valid.true_compass = 1;
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxHDT

		//	+++	decode &xxHDM
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_HDM);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->compass.mag_course_deg = (float)atof(str);
								nmea->compass.valid.mag_compass = 1;
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxHDT

		//	+++	decode &xxDPT
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_DPT);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->sounder.depth_m = (float)atof(str);
								nmea->sounder.valid.depth = 1;
							break;
							case 3:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->sounder.depth_offset_m = (float)atof(str);
								nmea->sounder.valid.depth_offset = 1;
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxDPT

		//	+++	decode &xxMTW
		do
		{
			end = false;
			found = strstr(nmea->buf, NMEA_MTW);
			if (found != NULL)
			{
				found -= 3;
				if (found[0] != '$')
					break;
				checksum = strstr(found, "*");
				if (checksum == NULL)
					break;
				checksum_read[0] = *(checksum + 1);
				checksum_read[1] = *(checksum + 2);
				memset(line, 0, sizeof(line));
				int cpy = checksum - found + 3;
				if (cpy > sizeof(line))
					break;
				strncpy(line, found, cpy);
				uint8_t c = nmea_checksum(line);
				int cc = 0;
				sscanf(checksum_read, "%X", &cc);
				if (cc == c)
				{
					char *str = strtok(line, ",*");
					uint8_t index = 2;
					while (str != NULL)
					{
						str = strtok(NULL, ",*");
						switch (index)
						{
							case 2:
								if (str[0] < '0' || str[0] > '9')
									break;
								nmea->sounder.temp_c = (float)atof(str);
							break;
							case 3:
								if (str[0] == 'C')
								{
									nmea->sounder.valid.temp = 1;
								}
								else if (str[0] == 'F')
								{
									nmea->sounder.temp_c = (nmea->sounder.temp_c - 32.0f) * 5.0f / 9.0f;
									nmea->sounder.valid.temp = 1;
								}
							break;
							default:
								end = true;
							break;
						}
						index++;
						if (end)
							break;
					}
					nmea->available = true;
				}
			}
		}
		while (0);
		//	---	decode $xxMTW

    memset(nmea->buf, 0, nmea->buf_size);
    nmea->buf_index = 0;
	}
	nmea->lock = false;
}
//###########################################################################################################################
bool nmea_available(nmea_t *nmea)
{
	return nmea->available;
}
//###########################################################################################################################
void nmea_available_reset(nmea_t *nmea)
{
	while (nmea->lock)
		nmea_delay(1);
	nmea->lock = true;
	memset(&nmea->gnss.valid, 0, sizeof(gnss_valid_t));
	memset(&nmea->compass.valid, 0, sizeof(compass_valid_t));
	memset(&nmea->sounder.valid, 0, sizeof(sounder_valid_t));
	memset(nmea->buf, 0, nmea->buf_size);
	nmea->buf_index = 0;
	nmea->available = false;
	nmea->lock = false;
}
//###########################################################################################################################
bool nmea_gnss_time_h(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.time == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.time_h;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_time_m(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.time == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.time_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_time_s(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.time == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.time_s;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_date_y(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.date == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.date_y;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_date_m(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.date == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.date_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_date_d(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.date == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.date_d;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_satellite(nmea_t *nmea, uint8_t *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.satellite == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.satellite;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_speed_kph(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.speed_knots == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.speed_knots * 1.852f;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_speed_knots(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.speed_knots == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.speed_knots;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_precision_m(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.precision == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.precision_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_course_deg(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.course == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.course_deg;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_latitude_deg(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.latitude == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.latitude_deg;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_longitude_deg(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.longitude == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.longitude_deg;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_gnss_altitude_m(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->gnss.valid.altitude == 1)
	{
		if (data != NULL)
			*data = nmea->gnss.altitude_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_compass_true_course_deg(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->compass.valid.true_compass == 1)
	{
		if (data != NULL)
			*data = nmea->compass.true_course_deg;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_compass_mag_course_deg(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->compass.valid.mag_compass == 1)
	{
		if (data != NULL)
			*data = nmea->compass.mag_course_deg;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_sounder_depth_m(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->sounder.valid.depth == 1)
	{
		if (data != NULL)
			*data = nmea->sounder.depth_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_sounder_depth_offset_m(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->sounder.valid.depth_offset == 1)
	{
		if (data != NULL)
			*data = nmea->sounder.depth_offset_m;
		return true;
	}
	return false;
}
//###########################################################################################################################
bool nmea_sounder_temp_c(nmea_t *nmea, float *data)
{
	if (nmea == NULL)
		return false;
	if (nmea->sounder.valid.temp == 1)
	{
		if (data != NULL)
			*data = nmea->sounder.temp_c;
		return true;
	}
	return false;
}
//###########################################################################################################################
