#include "nmeacomp.h"

typedef struct tagNCC_S {
	uint8_t c;
	char *s;
	uint8_t len;
} NCC_S;

typedef struct tagNCC_COMPR {
	uint16_t code;
	uint8_t len;
} NCC_COMPR;

typedef struct tagNCC_DECOMPR {
	uint8_t c;
	uint16_t code;
	uint8_t len;
	uint16_t cmpb;
} NCC_DECOMPR;

NCC_S g_ncc_s[]={
		'0',"010",3,
		',',"011",3,		
		'2',"1000",4,      
		'3',"1001",4,      
		'1',"1010",4,      
		'4',"1011",4,    
		'9',"1100",4,    
		'6',"11010",5,    
		'.',"11011",5, 

		'7',"00010",5,     
		'5',"00011",5,     

		'8',"111000",6,    
		'*',"111001",6,    
		'A',"111010",6,
		          
		'N',"001000",6,
		'E',"001001",6, 
		'M',"001010",6, 
		'K',"001011",6, 
		'T',"001100",6, 
		          
		'B',"1110110",7,
		'D',"1110111",7,
		'C',"1111000",7,
		'F',"1111001",7,
		'G',"1111010",7,
		          
		'H',"0011010",7,
		'I',"0011011",7,
		          
		'J',"11110110",8,
		'L',"11110111",8,  
		'O',"11111000",8,
		'P',"11111001",8,
		'Q',"11111010",8,
		'R',"11111011",8,
		'S',"11111100",8,
		'U',"11111101",8,
		'V',"11111110",8,
		'W',"11111111",8,
		          
		'X',"00111000",8,
		'Y',"00111001",8,
		'Z',"00111010",8,
		          
		'-',"001110110",9,				//unused fields
		'/',"001110111",9,
		'#',"001111000",9,
		'@',"001111001",9,
		'%',"001111010",9,
		          
		'^',"0011110110",10,
		'a',"0011110111",10,
		'b',"0011111000",10,
		'c',"0011111001",10,
		'd',"0011111010",10,
		'e',"0011111011",10,
		'f',"0011111100",10,
		'g',"0011111101",10,
		'h',"0011111110",10,
		'i',"0011111111",10,
		0,NULL,0
};

NCC_COMPR g_ncc_compr[256]={0};
NCC_DECOMPR g_ncc_decompr[256]={0};

unsigned char vcodes_chars[]={'0',',','2','3','1','4','9','6','.','7','5'};

uint16_t vcodes_byte[]={0x4000,0x6000,0x8000,0x9000,0xA000,0xB000,0xC000,0xD000,0x1000,0x1800};

unsigned char vcodes_len[]={3,3,4,4,4,4,4,5,5,5,5};

/**********************************************************************************/
int cmp(const void *a1, const void* a2)
{
	unsigned int *ia1=(unsigned int*)a1;
	unsigned int *ia2=(unsigned int*)a2;

	if(*(ia1+0) < *(ia2+0)) return(1);
	else if(*(ia1+0) > *(ia2+0)) return(-1);
	return(0);
}
/**********************************************************************************/
int freq(void)
{
	int rval=-1,cn;
	FILE *fp=NULL,*fw=NULL;
	unsigned char c,p;
	unsigned int i;
	unsigned int cf[256][2]={0},n;

//	if((fp=fopen("D:\\cipo\\robot\\pololu\\MiniGPS_1.4\\MiniGPS.nma",FOM_RO_BIN)) == NULL) goto end;
//	if((fp=fopen("D:\\cipo\\robot\\pololu\\MiniGPS_1.4\\nmea.log",FOM_RO_BIN)) == NULL) goto end;
	if((fp=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\MiniGPS.nma",FOM_RO_BIN)) == NULL) goto end;
//	if((fw=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\nmea.log",FOM_WO_BIN)) == NULL) goto end;

	cn=0;
	p='\0';
	while(fread((void*)&c,1,1,fp) == 1) {
		if((c == 0x0a) || (c == 0x0d)) {
			if(c == 0x0a) {
				fwrite((void*)&c,1,1,fw);
			}
			continue;
		}
		fwrite((void*)&c,1,1,fw);
		cf[c][0]++;
		cn++;

		if((p == 13) && (c != 10)) {
printf("%d\n",cn);
		}
		if((p != 13) && (c == 10)) {
printf("%d\n",cn);
		}

		p=c;
	}

	n=0;
	for(i=0;i < 256;i++) {
		cf[i][1]=i;
		if(cf[i][0] != 0) {
			n++;
			if(isprint(i)) {
				printf("%c - %u\n",i,cf[i][0]);
			} else {
				printf("%02x - %u\n",i,cf[i][0]);
			}
		}
	}
	printf("num of vals1: %u\n",n);

	qsort((void*)cf,256,2*sizeof(unsigned int),cmp);

	n=0;
	for(i=0;i < 256;i++) {
		if(cf[i][0] != 0) {
			n++;
			if(isprint(cf[i][1])) {
				printf("%c - %u\n",cf[i][1],cf[i][0]);
			} else {
				printf("%02x - %u\n",cf[i][1],cf[i][0]);
			}
		}
	}
	printf("num of vals2: %u\n",n);


	rval=0;

end:

	if(fp != NULL) fclose(fp);
	if(fw != NULL) fclose(fw);

	return(rval);
}
/**********************************************************************************/
int	nmea_compr(void)
{
	int rval=-1;
	FILE *fp=NULL,*fw=NULL;
	uint8_t ci,bp,*ptr;
	uint32_t bb,sb;

	if((fp=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\nmea1.log",FOM_RO_BIN)) == NULL) goto end;
	if((fw=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\nmea1.log.ncr",FOM_WO_BIN)) == NULL) goto end;

	bp=0;
	bb=0;
	ptr=(uint8_t*)&bb;
	ptr+=3;
	while(fread((void*)&ci,1,1,fp) == 1) {
		if((sb=g_ncc_compr[ci].code) == 0) {
			continue;
		}
		sb<<=(16-bp);
		bb|=sb;
		bp+=g_ncc_compr[ci].len;
		if(bp > 8) {
			if(fwrite((void*)ptr,1,1,fw) != 1) goto end;
			bp-=8;
			bb<<=8;
		}
	}
	while(bp > 0) {
		if(fwrite((void*)ptr,1,1,fw) != 1) goto end;
		if(bp > 8) {
			bp-=8;
		} else {
			break;
		}
		bb<<=8;
	}

	rval=0;

end:

	if(fp != NULL) fclose(fp);
	if(fw != NULL) fclose(fw);

	return(rval);
}
/**********************************************************************************/
int	nmea_decompr(void)
{
	int rval=-1,i,i0;
	FILE *fp=NULL,*fw=NULL;
	uint8_t ci,bp;
	uint16_t bb,sb;

	if((fp=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\nmea1.log.ncr",FOM_RO_BIN)) == NULL) goto end;
	if((fw=fopen("D:\\cipo\\robot\\pololu\\nmeacomp\\nmea1.log.ndecr",FOM_WO_BIN)) == NULL) goto end;

	bp=0;
	bb=0;
	i0=0;
	while(fread((void*)&ci,1,1,fp) == 1) {
		sb=ci<<8;
		sb>>=bp;
		bp+=8;
		bb|=sb;

		do {
			for(i=i0;;i++) {
				if(g_ncc_decompr[i].code == 0) {
					goto end;
				}
				if(g_ncc_decompr[i].len > bp) {
					i0=i;
					goto next;
				}
				if((bb&g_ncc_decompr[i].cmpb) == g_ncc_decompr[i].code) {
					if(fwrite((void*)&g_ncc_decompr[i].c,1,1,fw) != 1) goto end;
					bb<<=g_ncc_decompr[i].len;
					bp-=g_ncc_decompr[i].len;
					i0=0;
					break;
				}
			}
		} while(bp >= 3);

next:
		;
	}

	rval=0;

end:

	if(fp != NULL) fclose(fp);
	if(fw != NULL) fclose(fw);

	return(rval);
}
/**********************************************************************************/
int main(int argc, char *argv[])
{
	int rval=-1;
	long i;

//	freq();
//	goto end;

	for(i=0;;i++) {
		if(g_ncc_s[i].c == 0) break;
		g_ncc_compr[g_ncc_s[i].c].code=(uint16_t)strtol(g_ncc_s[i].s,NULL,2);
		g_ncc_compr[g_ncc_s[i].c].code=g_ncc_compr[g_ncc_s[i].c].code<<(16-g_ncc_s[i].len);
		g_ncc_compr[g_ncc_s[i].c].len=g_ncc_s[i].len;

		g_ncc_decompr[i].c=g_ncc_s[i].c;
		g_ncc_decompr[i].code=(uint16_t)strtol(g_ncc_s[i].s,NULL,2);
		g_ncc_decompr[i].code=g_ncc_decompr[i].code<<(16-g_ncc_s[i].len);
		g_ncc_decompr[i].len=g_ncc_s[i].len;
		if(g_ncc_decompr[i].len == 3) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1110000000000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 4) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111000000000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 5) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111100000000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 6) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111110000000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 7) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111111000000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 8) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111111100000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 9) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111111110000000",NULL,2);
		} else if(g_ncc_decompr[i].len == 10) {
			g_ncc_decompr[i].cmpb=(uint16_t)strtol("1111111110000000",NULL,2);
		} else goto end;
	}

	for(i=0;i < 256;i++) {
		printf("% 4ld. %c 0x%04x %d\n",i,(char)i,g_ncc_compr[i].code,g_ncc_compr[i].len);
	}

	if(nmea_compr() != 0) goto end;
	if(nmea_decompr() != 0) goto end;

	rval=0;

end:

	return(rval);
}
/**********************************************************************************/

/*
$GPGSA
$GPGSV
$GPGGA
$GPGLL
$GPRMC
$GPVTG
$GPZDA
$PMTKCHN
*/
/*
0 - 25411
, - 19197
2 - 9875
3 - 5814
1 - 5122
4 - 4299
9 - 3288
6 - 2744
. - 2718
7 - 2289
5 - 1919
8 - 1434
* - 1313
A - 764
N - 513
E - 493
M - 479
K - 102
T - 102
B - 87
D - 81
C - 80
F - 65
num of vals2: 23
*/

/*
00             switch
               
1.  010 					 0
2.  011						 ,
3.  1000           2
4.  1001           3
5.  1010           1
6.  1011           4
7.  1100           9
8.  11010          6
9.  11011          .
               
10. 00010          7
11. 00011          5
               
12. 111000         8
13. 111001         *
14. 111010         A
               
15. 001000         N
16. 001001         E
17. 001010         M
18. 001011         K
19. 001100         T
               
20. 1110110        B
21. 1110111        D
22. 1111000        C
23. 1111001        F
24. 1111010				 G				---
               
25. 0011010        H
26. 0011011        I
               
27. 11110110       J
28. 11110111       L
29. 11111000       O
30. 11111001       P
31. 11111010       Q
32. 11111011       R
33. 11111100       S
34. 11111101       U
35. 11111110       V
36. 11111111       W
               
37. 00111000       X
38. 00111001       Y
39. 00111010       Z
               
40. 001110110      
41. 001110111      
42. 001111000      
43. 001111001      
44. 001111010      
               
45. 0011110110     
46. 0011110111     
47. 0011111000     
48. 0011111001     
49. 0011111010     
50. 0011111011     
51. 0011111100     
52. 0011111101     
53. 0011111110     
54. 0011111111     
*/
