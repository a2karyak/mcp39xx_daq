#include <stdint.h>
#include <stdlib.h> // size_t

static const char board_ID[] = "ADM00523"; // TODO
static const char firmware_ID[] = "1.0.0"; // TODO

extern uint8_t channel_no[2];

extern int start_measurement(void);
extern void stop_measurement(void);
extern void read_data(unsigned n);
extern void write_reg(unsigned reg, uint32_t val);
extern uint32_t read_reg(unsigned reg);

static const char *read_dec(const char *p, const char *end, unsigned max, unsigned *v)
{
	if (p == end)
		return p;
	uint32_t val = 0;
	do
	{
		unsigned digit = *p - '0';
		if (digit > max
				|| val > (max - digit) / 10u)
		{
			return p;
		}
		val = val * 10 + digit;
	} while (++p != end && *p >= '0' && *p <= '9');
	*v = val;
	return p;
}

static const char *read_reg_index(const char *p, const char *end, unsigned *v)
{
	if (p == end || *p != '[')
		return p;
	p = read_dec(++p, end, 0xffffffffu, v);
	if (p == end || *p != ']')
		return p;
	return ++p;
}

static char *write_dec(char *p, unsigned v)
{
	uint8_t buf[9]; // TODO CONST
	unsigned i = 0;
	do
	{
		buf[i++] = v % 10u;
		v /= 10u;
	} while (v != 0);
	do
	{
		*p++ = buf[--i] + '0';
	} while (i != 0);
	return p;
}

static char *write_reg_index(char *p, unsigned reg)
{
	*p++ = '[';
	p = write_dec(p, reg);
	*p++ = ']';
	return p;
}

size_t run_command(const char *p, unsigned len,
		char *out_buf) // TODO: BUFOVF
{
	// TODO: incomplete commands are also accepted
	if (len == 0)
		return 0;
	char *s = out_buf;
	unsigned reg;
	unsigned val;
	const char *end = p + len;
	char cmd = *p++;
	switch (cmd)
	{
	case 'i':
		break;
	case 's':
		if (start_measurement() != 0)
			goto error;
		break;
	case 'p':
		stop_measurement();
		break;
	case '!':
		if (p == end)
			val = 1;
		else
			p = read_dec(p, end, 0xffffffffu, &val);
		read_data(val);
		return 0;
	case 'W': // write all
		goto error; // TODO
	case 'R': // read all
		goto error; // TODO
	case 'w': // write register
		p = read_reg_index(p, end, &reg);
		if (p == end || *p != ',')
			goto error;
		p = read_dec(++p, end, 0xffffffffu, &val); // TODO: various register sizes
		if (p != end)
			goto error;
		write_reg(reg, val);
		break;
	case 'r': // read register
		p = read_reg_index(p, end, &reg);
		if (p != end)
			goto error;
		val = read_reg(reg);
		*s++ = cmd;
		s = write_reg_index(s, reg);
		*s++ = ',';
		s = write_dec(s, val);
		*s++ = '\n';
		return s - out_buf;
	case 'v': // version
	case 't': // TEMPO_ADJ
		goto error; // TODO
	case 'C':
		p = read_dec(p, end, 7, &val);
		channel_no[0] = val;
		if (p != end)
		{
			if (*p != ',')
				goto error;
			p = read_dec(++p, end, 7, &val);
			channel_no[1] = val;
			if (p != end)
				goto error;
		}
		// fall through
	case 'c':
		*s++ = cmd;
		s = write_dec(s, channel_no[0]);
		*s++ = ',';
		s = write_dec(s, channel_no[1]);
		*s++ = '\n';
		return s - out_buf;
	default:
		return 0;
	}
	*s++ = cmd;
	*s++ = '0';
	*s++ = '\n';
	return s - out_buf;

error:
	*s++ = cmd;
	*s++ = '1';
	*s++ = '\n';
	return s - out_buf;
}
