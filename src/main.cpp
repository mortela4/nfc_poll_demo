#include <Arduino.h>

#include "SPI.h"

#include "rfal_platform/rfal_platform.h"


extern "C" {
#include "rfal_core/rfal_nfc.h"             // Includes all of "rfal_nfc[a|b|f|v].h", "rfal_isoDep.h" and "rfal_nfcDep.h".
#include "rfal_core/rfal_t2t.h"
#include "rfal_core/rfal_analogConfig.h"
}


// REFERENCE: https://www.st.com/resource/en/user_manual/um2890-rfnfc-abstraction-layer-rfal-stmicroelectronics.pdf
// ISO 15693
// ISO 1443(A)


#define REVERSE_BYTES(pData, nDataSize) \
  {unsigned char swap, *lo = ((unsigned char *)(pData)), *hi = ((unsigned char *)(pData)) + (nDataSize) - 1; \
  while (lo < hi) { swap = *lo; *lo++ = *hi; *hi-- = swap; }}



#define DEMO_NFCV_BLOCK_LEN           4     /*!< NFCV Block len                         */
                                                                                        
#define DEMO_NFCV_USE_SELECT_MODE     false /*!< NFCV // checknstrate select mode           */
#define DEMO_NFCV_WRITE_TAG           false /*!< NFCV // checknstrate Write Single Block    */
    
/* Definition of various Listen Mode constants */
#if defined(DEMO_LISTEN_MODE_TARGET) 
#define DEMO_LM_SEL_RES       0x40U         /*!<NFC-A SEL_RES configured for the NFC-DEP protocol    */
#define DEMO_LM_NFCID2_BYTE1  0x01U         /*!<NFC-F SENSF_RES configured for the NFC-DEP protocol  */ 
#define DEMO_LM_SC_BYTE1      0xFFU         /*!<NFC-F System Code byte 1                             */ 
#define DEMO_LM_SC_BYTE2      0xFFU         /*!<NFC-F System Code byte 2                             */ 
#define DEMO_LM_PAD0          0xFFU         /*!<NFC-F PAD0                                           */ 
#else
#define DEMO_LM_SEL_RES       0x20U         /*!<NFC-A SEL_RES configured for Type 4A Tag Platform    */
#define DEMO_LM_NFCID2_BYTE1  0x02U         /*!<NFC-F SENSF_RES configured for Type 3 Tag Platform   */ 
#define DEMO_LM_SC_BYTE1      0x12U         /*!<NFC-F System Code byte 1                             */ 
#define DEMO_LM_SC_BYTE2      0xFCU         /*!<NFC-F System Code byte 2                             */ 
#define DEMO_LM_PAD0          0x00U         /*!<NFC-F PAD0                                           */ 
#endif


/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

/* APDUs communication data */
#if RFAL_FEATURE_ISO_DEP_POLL
static uint8_t ndefSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };
static uint8_t ccSelectFile[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static uint8_t readBinary[] = { 0x00, 0xB0, 0x00, 0x00, 0x0F };

/* For a Payment application a Select PPSE would be needed: 
   ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 } */
#endif /* RFAL_FEATURE_ISO_DEP_POLL */

#if RFAL_FEATURE_NFC_DEP
/* P2P communication data */
static uint8_t ndefLLCPSYMM[] = {0x00, 0x00};
static uint8_t ndefInit[] = {0x05, 0x20, 0x06, 0x0F, 0x75, 0x72, 0x6E, 0x3A, 0x6E, 0x66, 0x63, 0x3A, 0x73, 0x6E, 0x3A, 0x73, 0x6E, 0x65, 0x70, 0x02, 0x02, 0x07, 0x80, 0x05, 0x01, 0x02};
static uint8_t ndefUriSTcom[] = {0x13, 0x20, 0x00, 0x10, 0x02, 0x00, 0x00, 0x00, 0x19, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x12, 0x55, 0x00, 0x68, 0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 0x73, 0x74, 0x2e, 0x63, 0x6f, 0x6d};
#endif /* RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
#if RFAL_SUPPORT_MODE_LISTEN_NFCA
/* NFC-A CE config */
/* 4-byte UIDs with first byte 0x08 would need random number for the subsequent 3 bytes.
 * 4-byte UIDs with first byte 0x*F are Fixed number, not unique, use for this // check
 * 7-byte UIDs need a manufacturer ID and need to assure uniqueness of the rest.*/
static uint8_t ceNFCA_NFCID[]     = {0x5F, 'S', 'T', 'M'};    /* =_STM, 5F 53 54 4D NFCID1 / UID (4 bytes) */
static uint8_t ceNFCA_SENS_RES[]  = {0x02, 0x00};             /* SENS_RES / ATQA for 4-byte UID            */
static uint8_t ceNFCA_SEL_RES     = DEMO_LM_SEL_RES;          /* SEL_RES / SAK                             */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

static uint8_t ceNFCF_nfcid2[]     = {DEMO_LM_NFCID2_BYTE1, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
  /* NFC-F CE config */
static uint8_t ceNFCF_SC[]         = {DEMO_LM_SC_BYTE1, DEMO_LM_SC_BYTE2};
static uint8_t ceNFCF_SENSF_RES[]  = {0x01,                                                       /* SENSF_RES                                */
                                  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,                 /* NFCID2                                   */
                                  DEMO_LM_PAD0, DEMO_LM_PAD0, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00, /* PAD0, PAD1, MRTIcheck, MRTIupdate, PAD2  */
                                  0x00, 0x00 };                                                   /* RD                                       */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */


static rfalNfcDiscoverParam discParam;      // NFC discovery parameters.

bool multiSel = false;                      // Handling of multiple NFC-tags simultaneously.


// Helpers:
static char UID_hex_string[40] = {0};       // Safely hold up to NFCID3 values, i.e. 10-byte (requires 21-byte char-arr)

static char *hex2str(uint8_t *number, uint8_t length)
{
	uint8_t aux_1 = 0;
	uint8_t aux_2 = 0;

	for (uint8_t i=0; i < length; i++)
	{
		aux_1 = number[i] / 16;
		aux_2 = number[i] % 16;

		if (aux_1 < 10)
		{
			UID_hex_string[2*i] = aux_1 + '0';
		}
		else
        {
			UID_hex_string[2*i] = aux_1 + ('A' - 10);
		}

		if (aux_2 < 10)
        {
			UID_hex_string[2*i+1] = aux_2 + '0';
		}
		else
        {
			UID_hex_string[2*i+1] = aux_2 + ('A' - 10);
		}
	} 

	UID_hex_string[length*2] = '\0';

    return((char *)UID_hex_string);
}

struct rfalStateToDescription
{
    rfalNfcState state;
    String desc;
};

#define NUM_RFAL_NFC_STATES     16

static const struct rfalStateToDescription stateToDesc[NUM_RFAL_NFC_STATES] = 
{
    {
        .state = RFAL_NFC_STATE_NOTINIT,
        .desc = "NOT_INITIALIZED",
    },
    {
        .state = RFAL_NFC_STATE_IDLE,
        .desc = "IDLE",
    },
    {
        .state = RFAL_NFC_STATE_START_DISCOVERY,
        .desc = "START_DISCOVERY",
    },
    {
        .state = RFAL_NFC_STATE_WAKEUP_MODE,
        .desc = "WKUP_MODE",
    },
    {
        .state = RFAL_NFC_STATE_POLL_TECHDETECT,
        .desc = "POLL_TECH_DETECT",
    },
    {
        .state = RFAL_NFC_STATE_POLL_COLAVOIDANCE,
        .desc = "POLL_COLL_AVOIDANCE",
    },
    {
        .state = RFAL_NFC_STATE_POLL_SELECT,
        .desc = "POLL_SELECT",
    },
    {
        .state = RFAL_NFC_STATE_POLL_ACTIVATION,
        .desc = "POLL_ACTIVATION",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_TECHDETECT,
        .desc = "LISTEN_TECH_DETECT",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_COLAVOIDANCE,
        .desc = "LISTEN_COLL_AVOIDANCE",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_ACTIVATION,
        .desc = "LISTEN_ACTIVATION",
    },
    {
        .state = RFAL_NFC_STATE_LISTEN_SLEEP,
        .desc = "LISTEN_SLEEP",
    },
    {
        .state = RFAL_NFC_STATE_ACTIVATED,
        .desc = "STATE_ACTIVATED",
    },
    {
        .state = RFAL_NFC_STATE_DATAEXCHANGE,
        .desc = "DATA_EXCHANGE_STARTED",
    },
    {
        .state = RFAL_NFC_STATE_DATAEXCHANGE_DONE,
        .desc = "DATA_EXCHANGE_DONE",
    },
    {
        .state = RFAL_NFC_STATE_DEACTIVATION,
        .desc = "DE-ACTIVATION",
    },
};

static const String nonStateDesc = "<invalid state>";

static String state_description(rfalNfcState rfalState)
{
    String stateDesc = nonStateDesc;

    for (int i = 0; i < NUM_RFAL_NFC_STATES; i++)
    {
        struct rfalStateToDescription *entry = (struct rfalStateToDescription *)&stateToDesc[i];

        if (rfalState == entry->state)
        {
            stateDesc = entry->desc;
        }
    }

    return(stateDesc);
}


static void demoNfcv( rfalNfcvListenDevice *nfcvDev )
{
#if RFAL_FEATURE_NFCV
    
    ReturnCode            err;
    uint16_t              rcvLen;
    uint8_t               blockNum = 1;
    uint8_t               rxBuf[ 1 + DEMO_NFCV_BLOCK_LEN + RFAL_CRC_LEN ];                        /* Flags + Block Data + CRC */
    uint8_t               *uid; 
    uint8_t               reqFlag;
#if DEMO_NFCV_WRITE_TAG
    uint8_t               wrData[DEMO_NFCV_BLOCK_LEN] = { 0x11, 0x22, 0x33, 0x99 };             /* Write block example */
#endif /* DEMO_NFCV_WRITE_TAG */
              

    uid     = nfcvDev->InvRes.UID;
    reqFlag = RFAL_NFCV_REQ_FLAG_DEFAULT;
    
    #if DEMO_NFCV_USE_SELECT_MODE
        /*
        * Activate selected state
        */
        err = rfalNfcvPollerSelect( reqFlag, nfcvDev->InvRes.UID );
        platformLog(" Select %s \r\n", (err != RFAL_ERR_NONE) ? "FAIL (revert to addressed mode)": "OK" );
        if( err == RFAL_ERR_NONE )
        {
            reqFlag = (RFAL_NFCV_REQ_FLAG_DEFAULT | RFAL_NFCV_REQ_FLAG_SELECT);
            uid     = NULL;
        }
    #endif /* DEMO_NFCV_USE_SELECT_MODE */

    /*
    * Read block using Read Single Block command
    * with addressed mode (uid != NULL) or selected mode (uid == NULL)
    */
    err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
    Serial0.println(" Read Block:");
    Serial0.println( (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:");
    Serial0.println( (err != RFAL_ERR_NONE) ? "" : hex2str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
 
    #if DEMO_NFCV_WRITE_TAG /* Writing example */
        err = rfalNfcvPollerWriteSingleBlock(reqFlag, uid, blockNum, wrData, sizeof(wrData));
        platformLog(" Write Block: %s Data: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", hex2Str( wrData, DEMO_NFCV_BLOCK_LEN) );
        err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
        platformLog(" Read Block: %s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:", (err != RFAL_ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));
    #endif /* DEMO_NFCV_WRITE_TAG */
        
#endif /* RFAL_FEATURE_NFCV */
}



// RFAL notification callback.
static void rfalNotifyCb( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;
    
    Serial0.print("RFAL-notify: RFAL NFC-state changed! New state: ");
    Serial0.println( state_description(rfalNfcGetState()) );

    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        Serial0.println("Wake Up mode started ...");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        if( discParam.wakeupEnabled )
        {
            Serial0.println("Wake Up mode terminated. Polling for devices ...");
        }
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Check if in case of multiple devices, selection is already attempted */
        if( (!multiSel) )
        {
            multiSel = true;
            /* Multiple devices were found, activate first of them */
            rfalNfcGetDevicesFound( &dev, &devCnt );
            rfalNfcSelect( 0 );
            
            Serial0.println("Multiple Tags detected! Activating first device. No. of devices: ");
            Serial0.println(devCnt);
        }
        else
        {
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
            Serial0.println("RFAL-notify: de-activating NFC again ...");
        }
    }
    else if( st == RFAL_NFC_STATE_START_DISCOVERY )
    {
        /* Clear multiple device selection flag */
        multiSel = false;
    }
}


void setup() 
{
    Serial0.begin(115200);
    Serial0.println("Init ...");
    
    spi_init();

    // NFC:
    ReturnCode ret = rfalNfcInitialize();      // WAS: 'rfalInitialize()' - but this function is NOT setting NFC-state!!
    
    if (RFAL_ERR_NONE != ret)
    {
        Serial0.println("ERROR: NFC subsystem init failed!\nNFC subsystem init return code:");
        Serial0.println(ret);
        while(1)
        {
            vTaskDelay(1000);
        }
    }

    Serial0.println("NFC subsystem initialized OK...");

    // NFC subsystem init OK - go on to set up polling: 
    rfalNfcDefaultDiscParams( &discParam );
    
    discParam.devLimit      = 1U;

    discParam.wakeupEnabled = false;                        // NOTE: possibly - a better solution is to let user-button switch this TRUE/FALSE during run-time(??).
    
    SMEMCPY( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
    SMEMCPY( &discParam.GB, GB, sizeof(GB) );
    discParam.GBLen         = sizeof(GB);
    discParam.p2pNfcaPrio   = true;

    discParam.notifyCb             = rfalNotifyCb;
    discParam.totalDuration        = 3000U;                         /* Duration of POLL + LISTEN cycle = 1000ms = 1sec. */
    discParam.techs2Find           = RFAL_NFC_TECH_NONE;          /* For the // check, enable the NFC Technologies based on RFAL Feature switches */


#if RFAL_FEATURE_NFCA
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_A;
#endif /* RFAL_FEATURE_NFCA */

#if RFAL_FEATURE_NFCB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_B;
#endif /* RFAL_FEATURE_NFCB */

#if RFAL_FEATURE_NFCF
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_F;
#endif /* RFAL_FEATURE_NFCF */

#if RFAL_FEATURE_NFCV
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_V;
#endif /* RFAL_FEATURE_NFCV */

#if RFAL_FEATURE_ST25TB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_ST25TB;
#endif /* RFAL_FEATURE_ST25TB */

#ifdef ST25R95
        discParam.isoDepFS           = RFAL_ISODEP_FSXI_128;          /* ST25R95 cannot support 256 bytes of data block */
#endif /* ST25R95 */

#if RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP
        discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE
        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
#endif /* RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE */


#if DEMO_CARD_EMULATION_ONLY
        discParam.totalDuration        = 60000U;              /* 60 seconds */
        discParam.techs2Find           = RFAL_NFC_TECH_NONE;  /* Overwrite any previous poller modes */
#endif /* DEMO_CARD_EMULATION_ONLY */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
        // checkCeInit( ceNFCF_nfcid2 );
    
#if RFAL_SUPPORT_MODE_LISTEN_NFCA
        /* Set configuration for NFC-A CE */
        SMEMCPY( discParam.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN );     /* Set SENS_RES / ATQA */
        SMEMCPY( discParam.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04 );           /* Set NFCID / UID */
        discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;                                  /* Set NFCID length to 7 bytes */
        discParam.lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                        /* Set SEL_RES / SAK */

        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
        /* Set configuration for NFC-F CE */
        SMEMCPY( discParam.lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN );                 /* Set System Code */
        SMEMCPY( &ceNFCF_SENSF_RES[RFAL_NFCF_CMD_LEN], ceNFCF_nfcid2, RFAL_NFCID2_LEN );     /* Load NFCID2 on SENSF_RES */
        SMEMCPY( discParam.lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN );  /* Set SENSF_RES / Poll Response */

        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_F;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */

        ret = rfalNfcDiscover( &discParam );
        if (RFAL_ERR_NONE != ret)
        {
            Serial0.println("ERROR: NFC subsystem init failed!\nNFC subsystem init return code:");
            Serial0.println(ret);
            while(1)
            {
                vTaskDelay(1000);
            }
        }

        Serial0.println("Setup of discovery parameters for NFC-scan OK ...");
        Serial0.print("NFC state BEFORE: ");
        Serial0.println( state_description(rfalNfcGetState()) );
        rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_DISCOVERY);
        Serial0.print("NFC state AFTER: ");
        Serial0.println( state_description(rfalNfcGetState()) );
}


void loop() 
{
    static rfalNfcDevice *nfcDevice;

    Serial0.println("Worker - start scan ...");

    // put your main code here, to run repeatedly:
    rfalNfcWorker(); //TODO: put in a separate thread. NOTE: was 'rfalWorker()'.

    if( rfalNfcIsDevActivated( rfalNfcGetState() ) )
    {
        Serial0.println("ST25R3911B NFC-xcvr active - polling ...");

        rfalNfcGetActiveDevice( &nfcDevice );
        
        switch( nfcDevice->type )
        {
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCA:
            
                platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
                switch( nfcDevice->dev.nfca.type )
                {
                    case RFAL_NFCA_T1T:
                        //platformLog("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        Serial0.print("ISO14443A/Topaz (NFC-A T1T) TAG found. UID:");
                        break;
                    
                    case RFAL_NFCA_T4T:
                        Serial0.print("NFCA Passive ISO-DEP device found. UID: ");
                    
                        // checkAPDU();
                        break;
                    
                    case RFAL_NFCA_T4T_NFCDEP:
                    case RFAL_NFCA_NFCDEP:
                        Serial0.print("NFCA Passive P2P device found. NFCID: ");
                        
                        // checkP2P( nfcDevice );
                        break;
                        
                    default:
                        Serial0.print("ISO14443A/NFC-A card found. UID: ");
                        
                        // checkT2t();
                        break;
                }

                Serial0.println(hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCB:
                
                platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
                Serial0.print("ISO14443B/NFC-B card found. UID: ");
                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
            
                if( rfalNfcbIsIsoDepSupported( &nfcDevice->dev.nfcb ) )
                {
                    // checkAPDU();
                }
                break;
                
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCF:
                
                platformLedOn(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
                if( rfalNfcfIsNfcDepSupported( &nfcDevice->dev.nfcf ) )
                {
                    Serial0.print("NFCF Passive P2P device found. NFCID: ");
                    // checkP2P( nfcDevice );
                }
                else
                {
                    Serial0.print("Felica/NFC-F card found. UID: ");
                    
                    // checkNfcf( &nfcDevice->dev.nfcf );
                }

                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_NFCV:
                {
                    uint8_t devUID[RFAL_NFCV_UID_LEN];
                    
                    platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
                    
                    SMEMCPY( devUID, nfcDevice->nfcid, nfcDevice->nfcidLen );   /* Copy the UID into local var */
                    REVERSE_BYTES( devUID, RFAL_NFCV_UID_LEN );                 /* Reverse the UID for display purposes */
                    Serial0.print("ISO15693/NFC-V card found. UID: ");
                    Serial0.println( hex2str(devUID, RFAL_NFCV_UID_LEN) );      /* NOTE: previous standards had 7-byte(=56-bit) UID, while NFC-V is 8-byte(=64-bit)!! */
                    
                    //checkNfcv( &nfcDevice->dev.nfcv );
                    demoNfcv( &nfcDevice->dev.nfcv );
                }
                break;
                
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_ST25TB:
                
                platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
                Serial0.print("ST25TB card found. UID: ");
                Serial0.println( hex2str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_LISTEN_TYPE_AP2P:
            case RFAL_NFC_POLL_TYPE_AP2P:
                
                platformLedOn(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
                Serial0.print("NFC Active P2P device found. NFCID3: ");
                Serial0.println( hex2str(nfcDevice->nfcid, nfcDevice->nfcidLen) );
            
                // checkP2P( nfcDevice );
                break;
            
            /*******************************************************************************/
            case RFAL_NFC_POLL_TYPE_NFCA:
            case RFAL_NFC_POLL_TYPE_NFCF:
                
                platformLedOn( ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PORT : PLATFORM_LED_F_PORT), 
                                ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PIN  : PLATFORM_LED_F_PIN)  );
                Serial0.print("Activated in CE ");
                Serial0.print( (nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? "NFC-A" : "NFC-F" );
                Serial0.println(" mode."); 

                if( nfcDevice->rfInterface == RFAL_NFC_INTERFACE_NFCDEP )
                {
                    Serial0.println("Interface = NFCDEP (P2P-capable) ..."); 
                    // checkP2P( nfcDevice );
                }
                else
                {
                    // checkCE( nfcDevice );
                }
                break;
            
            /*******************************************************************************/
            default:
                Serial0.println("WARN: unrecognized NFC device!(??)");
                break;
        }
        
         //rfalNfcDeactivate(RFAL_NFC_DEACTIVATE_IDLE);
        rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
    }
    else
    {
        Serial0.println("INFO: ST25R3911B NFC-xcvr deactivated ...");
    }

    //rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );   // TODO: assess - is 'rfalNfcDeactivate()' required to be run after a scan??? (then re-activate w. ''rfalNfcDeactivate()' ??)


    vTaskDelay(5000);
}


