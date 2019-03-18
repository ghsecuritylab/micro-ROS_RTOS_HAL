#include "microros_spi_hal.h"

#ifdef nuttx
	struct spi_dev_s *spi_aux;

	static void nuttx_spi_lock(bool state){
		//This functions blocks the bus like a mutex
		if(state){
			(void)SPI_LOCK(spi_aux, true);

			SPI_SETMODE(spi_aux, SPIDEV_MODE0);
			SPI_SETBITS(spi_aux, 8);
			(void)SPI_HWFEATURES(spi_aux, 0);
			//Esta es la frequencia para este sensor en particular, pero es encesario pensar otra manera de
			//obtener este dato.
			(void)SPI_SETFREQUENCY(spi_aux, 5000000);
		}
		else{
			(void)SPI_LOCK(spi_aux, false);
		}
	}

	void nuttx_spi_write(uint8_t regaddr, int8_t regval,uint8_t len ){
		nuttx_spi_lock(true);
		if(len ==1){
			//Send
			SPI_SEND(spi_aux, regaddr);
			SPI_SEND(spi_aux, regval);
		}
		else if (len > 1){
			SPI_SEND(spi_aux, regaddr );
			SPI_SNDBLOCK(spi_aux, regval, len);
		}
		nuttx_spi_lock(false);

	}

	uint32_t nuttx_spi_read(uint8_t regaddr,uint8_t len ){
		uint32_t regval;
		uint8_t address = (0x80 | (regaddr )); //?? es solo de este driver

		nuttx_spi_lock(true); //Aqui no se si sera correcto hace el bloqueo del bus o antes del CS
		if(len==1){
			SPI_SEND(spi_aux, address);
			regval = SPI_SEND(spi_aux, 0);
		}
		else{
			  SPI_SEND(spi_aux, address);
				SPI_RECVBLOCK(spi_aux, &regval, len);

				if(len==2){
					uint16_t msb, lsb;

					msb = (regval & 0xFF);
					lsb = (regval & 0xFF00) >> 8;

					regval = (msb << 8) | lsb;
				}
				else if(len==3){
					uint16_t v1, v2,v3;

					v1=(regval & 0xFF);
					v2=(regval & 0xFF00) >>8;
					v3=(regval & 0xFFFF00) >> 16;


					regval = (v1 << 16) | (v2 << 8) | v3;
				}
		}
		nuttx_spi_lock(false);

		return regval;

	}

	uint32_t nuttx_spi_hal_aux(struct spi_dev_s *spi){
	  //This funciton is to obtain the data of the connection form the driver an can use local in this file
	  memcpy(&spi_aux,&spi,sizeof(spi));
	}
#endif
