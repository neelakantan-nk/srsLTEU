/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/select.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>

#include "srslte/srslte.h"


#define UE_CRNTI 0x1234


#ifndef DISABLE_RF
#include "srslte/rf/rf.h"
srslte_rf_t rf; 
srslte_rf_t rf_pcc; // RF object for PCC 
#else
#warning Compiling pdsch_ue with no RF support
#endif

char *output_file_name = NULL;

#define LEFT_KEY  68
#define RIGHT_KEY 67
#define UP_KEY    65
#define DOWN_KEY  66

srslte_cell_t cell = {
  25,            // nof_prb
  1,            // nof_ports
  0,            // bw idx 
  0,            // cell_id
  SRSLTE_CP_NORM,       // cyclic prefix
  SRSLTE_PHICH_R_1,          // PHICH resources      
  SRSLTE_PHICH_NORM    // PHICH length
};
  
int net_port = -1; // -1 generates random dataThat means there is some problem sending samples to the device 
int net_port_pcc = -1; // Default port for PCC 

uint32_t cfi=3;
uint32_t mcs_idx = 1, last_mcs_idx = 1; 
//uint32_t mcs_idx_pcc = 1; // MCS index for PCC 
int nof_frames = -1;

// TODO: decide if these parameters need some default value
char *rf_args = "";
char *rf_args_pcc = ""; // RF Args for PCC 

float rf_amp = 0.8, rf_gain = 70.0, rf_freq = 2400000000; 

float rf_amp_pcc = 0.8, rf_gain_pcc = 70.0, rf_freq_pcc = 1800000000; //Parameters for PCC RF chain 

int sf_start = 0; //Default first active SF 
int sf_end = 10; //Default last active SF 

bool null_file_sink=false; 
srslte_filesink_t fsink;
srslte_ofdm_t ifft;
srslte_pbch_t pbch; 
srslte_pcfich_t pcfich; 
srslte_pdcch_t pdcch;
srslte_pdsch_t pdsch;
srslte_pdsch_cfg_t pdsch_cfg; 
srslte_softbuffer_tx_t softbuffer; 

srslte_regs_t regs;
srslte_ra_dl_dci_t ra_dl;  

srslte_softbuffer_tx_t softbuffer_pcc; 

cf_t *sf_buffer = NULL, *output_buffer = NULL;
cf_t *sf_buffer_pcc = NULL, *output_buffer_pcc = NULL; 

int sf_n_re, sf_n_samples;

pthread_t net_thread; 
pthread_t net_thread_pcc; 
void *net_thread_fnc(void *arg);
sem_t net_sem; 
sem_t net_sem_pcc; 
bool net_packet_ready = false; 
bool net_packet_ready_pcc = false;
srslte_netsource_t net_source; 
srslte_netsource_t net_source_pcc; 
srslte_netsink_t net_sink; 
srslte_netsink_t net_sink_pcc;

int prbset_num = 1, last_prbset_num = 1; 
int prbset_orig = 0; 


// TODO: always update the help for better management
// TODO: Do we need parameter for duty cycle?
void usage(char *prog) {
  printf("Usage: %s [ablgofsenmpuv]\n", prog);
#ifndef DISABLE_RF
  printf("\t-a RF args PCC [Default %s]\n", rf_args_pcc);
  printf("\t-b RF args SCC [Default %s]\n", rf_args);
  printf("\t-l RF TX gain PCC [Default %.2f]\n", rf_gain_pcc);
  printf("\t-g RF TX gain SCC [Default %.2f dB]\n", rf_gain);
  printf("\t-o RF TX frequency PCC [Default %.1f MHz]\n", rf_freq_pcc / 1000000);
  printf("\t-f RF TX frequency SCC [Default %.1f MHz]\n", rf_freq / 1000000);
  printf("\t-s SCC - First active Sub Frame [Default %s]\n", sf_start); 
  printf("\t-e SCC - Last active Sub Frame [Default %s]\n", sf_end); 
#else
  printf("\t   RF is disabled.\n");
#endif
 // printf("\t-n MCS index PCC [Default %d]\n", mcs_idx_pcc);
  printf("\t-m MCS index (common for PCC and SCC) [Default %d]\n", mcs_idx);
  printf("\t-p listen TCP port for input data PCC [Default %d]\n", net_port_pcc);
  printf("\t-u listen TCP port for input data SCC (-1 is random) [Default %d]\n", net_port);
  printf("\t-v [set srslte_verbose to debug, default none]\n");
}

// TODO: some parameters are same for both RF front ends.
// Ideally they should be decoupled
void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "ablgofsenmpuv")) != -1) {
    switch (opt) {
    case 'a':
      rf_args_pcc = argv[optind];
      break;
    case 'b':
      rf_args = argv[optind]; 
      break; 
    case 'g':
      rf_gain = atof(argv[optind]);
      break;
    case 'l':
      rf_gain_pcc = atof(argv[optind]);
      break;
    case 'f':
      rf_freq = atof(argv[optind]);
      break;
    case 's': 
      sf_start = atoi(argv[optind]);
      break;
    case 'e': 
      sf_end = atoi(argv[optind]);
      break; 
    case 'o':
      rf_freq_pcc = atof(argv[optind]);;
      break;
    case 'm':
      mcs_idx = atoi(argv[optind]);
      break;
    case 'u':
      net_port = atoi(argv[optind]);
      break;
    //case 'n':
    //  mcs_idx_pcc = atoi(argv[optind]);
    //  break;
    case 'p':
      net_port_pcc = atoi(argv[optind]);
      break;
    case 'v':
      srslte_verbose++;
      break;
    default:
      usage(argv[0]);
      exit(-1);
    }
  }
#ifdef DISABLE_RF
  if (!output_file_name) {
    usage(argv[0]);
    exit(-1);
  }
#endif
}

void base_init() {
  
  /* init memory */
  sf_buffer = srslte_vec_malloc(sizeof(cf_t) * sf_n_re);
  if (!sf_buffer) {
    perror("malloc");
    exit(-1);
  }
  output_buffer = srslte_vec_malloc(sizeof(cf_t) * sf_n_samples);
  if (!output_buffer) {
    perror("malloc");
    exit(-1);
  }
 sf_buffer_pcc = srslte_vec_malloc(sizeof(cf_t) * sf_n_re);
  if (!sf_buffer_pcc) {
    perror("malloc");
    exit(-1);
  }
  output_buffer_pcc = srslte_vec_malloc(sizeof(cf_t) * sf_n_samples);
  if (!output_buffer_pcc) {
    perror("malloc");
    exit(-1);
  }

  /* open file or USRP */
  if (output_file_name) {
    if (strcmp(output_file_name, "NULL")) {
      if (srslte_filesink_init(&fsink, output_file_name, SRSLTE_COMPLEX_FLOAT_BIN)) {
        fprintf(stderr, "Error opening file %s\n", output_file_name);
        exit(-1);
      }      
      null_file_sink = false; 
    } else {
      null_file_sink = true; 
    }
  } else {
#ifndef DISABLE_RF
    printf("Opening RF device...\n");
    if (srslte_rf_open(&rf, rf_args)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    printf("Opening RF device for PCC...\n"); 
    if (srslte_rf_open(&rf_pcc, rf_args_pcc)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }    
#else
    printf("Error RF not available. Select an output file\n");
    exit(-1);
#endif
  }
  
  if (net_port > 0) {
    if (srslte_netsource_init(&net_source, "0.0.0.0", net_port, SRSLTE_NETSOURCE_TCP)) {
      fprintf(stderr, "Error creating input UDP socket for SCC at port %d\n", net_port);
      exit(-1);
    }
    if (null_file_sink) {
      if (srslte_netsink_init(&net_sink, "127.0.0.1", net_port+1, SRSLTE_NETSINK_TCP)) {
        fprintf(stderr, "Error sink\n");
        exit(-1);
      } 
    }
     if (sem_init(&net_sem, 0, 1)) {
      perror("sem_init");
      exit(-1);
    } 
    } 

  if (net_port_pcc > 0) {
     if (srslte_netsource_init(&net_source_pcc, "0.0.0.0", net_port_pcc, SRSLTE_NETSOURCE_TCP)) {
         fprintf(stderr, "Error creating input UDP socket for PCC at port %d\n", net_port);
          exit(-1);
       }
     if (null_file_sink) {
      if (srslte_netsink_init(&net_sink_pcc, "127.0.0.1", net_port_pcc+1, SRSLTE_NETSINK_TCP)) {
        fprintf(stderr, "Error sink\n");
        exit(-1);
      }  
    }
    if (sem_init(&net_sem_pcc, 0, 1)) {
      perror("sem_init");
      exit(-1);
    }
  }

  /* create ifft object */
  if (srslte_ofdm_tx_init(&ifft, SRSLTE_CP_NORM, cell.nof_prb)) {
    fprintf(stderr, "Error creating iFFT object\n");
    exit(-1);
  }
  srslte_ofdm_set_normalize(&ifft, true);
  if (srslte_pbch_init(&pbch, cell)) {
    fprintf(stderr, "Error creating PBCH object\n");
    exit(-1);
  }

  if (srslte_regs_init(&regs, cell)) {
    fprintf(stderr, "Error initiating regs\n");
    exit(-1);
  }

  if (srslte_pcfich_init(&pcfich, &regs, cell)) {
    fprintf(stderr, "Error creating PBCH object\n");
    exit(-1);
  }

  if (srslte_regs_set_cfi(&regs, cfi)) {
    fprintf(stderr, "Error setting CFI\n");
    exit(-1);
  }

  if (srslte_pdcch_init(&pdcch, &regs, cell)) {
    fprintf(stderr, "Error creating PDCCH object\n");
    exit(-1);
  }

  if (srslte_pdsch_init(&pdsch, cell)) {
    fprintf(stderr, "Error creating PDSCH object\n");
    exit(-1);
  }
  
  srslte_pdsch_set_rnti(&pdsch, UE_CRNTI);
  
  if (srslte_softbuffer_tx_init(&softbuffer, cell.nof_prb)) {
    fprintf(stderr, "Error initiating soft buffer\n");
    exit(-1);
  }

  if (srslte_softbuffer_tx_init(&softbuffer_pcc, cell.nof_prb)) {
    fprintf(stderr, "Error initiating soft buffer PCC\n");
    exit(-1);
  }

}

void base_free() {

  srslte_softbuffer_tx_free(&softbuffer);
  srslte_pdsch_free(&pdsch);
  srslte_pdcch_free(&pdcch);
  srslte_regs_free(&regs);
  srslte_pbch_free(&pbch);

  srslte_ofdm_tx_free(&ifft);

  if (sf_buffer) {
    free(sf_buffer);
  }
  if (output_buffer) {
    free(output_buffer);
  }
 if (sf_buffer_pcc) {
    free(sf_buffer_pcc);
  }
  if (output_buffer_pcc) {
    free(output_buffer_pcc);
  }

  if (output_file_name) {
    if (!null_file_sink) {
      srslte_filesink_free(&fsink);      
    }
  } else {
#ifndef DISABLE_RF
    srslte_rf_close(&rf);
    srslte_rf_close(&rf_pcc); 
#endif
  }
  
  if (net_port > 0) {
    srslte_netsource_free(&net_source);
    sem_close(&net_sem);
  }  

  if (net_port_pcc > 0) {
    srslte_netsource_free(&net_source_pcc);
    sem_close(&net_sem_pcc);
  }  

}


bool go_exit = false; 
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}



unsigned int
reverse(register unsigned int x)
{
    x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
    x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
    x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
    x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
    return((x >> 16) | (x << 16));

}

uint32_t prbset_to_bitmask() {
  uint32_t mask=0;
  int nb = (int) ceilf((float) cell.nof_prb / srslte_ra_type0_P(cell.nof_prb));
  for (int i=0;i<nb;i++) {
    if (i >= prbset_orig && i < prbset_orig + prbset_num) {
      mask = mask | (0x1<<i);     
    }
  }
  return reverse(mask)>>(32-nb); 
}

int update_radl() {
  
  bzero(&ra_dl, sizeof(srslte_ra_dl_dci_t));
  ra_dl.harq_process = 0;
  ra_dl.mcs_idx = mcs_idx;
  ra_dl.ndi = 0;
  ra_dl.rv_idx = 0;
  ra_dl.alloc_type = SRSLTE_RA_ALLOC_TYPE0;
  ra_dl.type0_alloc.rbg_bitmask = prbset_to_bitmask();

  srslte_ra_pdsch_fprint(stdout, &ra_dl, cell.nof_prb);
  srslte_ra_dl_grant_t dummy_grant; 
  srslte_ra_nbits_t dummy_nbits;
  srslte_ra_dl_dci_to_grant(&ra_dl, cell.nof_prb, UE_CRNTI, &dummy_grant);
  srslte_ra_dl_grant_to_nbits(&dummy_grant, cfi, cell, 0, &dummy_nbits);
  srslte_ra_dl_grant_fprint(stdout, &dummy_grant);
  printf("Type new MCS index and press Enter: "); fflush(stdout);
 
  return 0; 
}

/* Read new MCS from stdin */
int update_control() {
  char input[128];
  
  fd_set set; 
  FD_ZERO(&set);
  FD_SET(0, &set);
  
  struct timeval to; 
  to.tv_sec = 0; 
  to.tv_usec = 0; 

  int n = select(1, &set, NULL, NULL, &to);
  if (n == 1) {
    // stdin ready
    if (fgets(input, sizeof(input), stdin)) {
      if(input[0] == 27) {
        switch(input[2]) {
          case RIGHT_KEY:
            if (prbset_orig  + prbset_num < (int) ceilf((float) cell.nof_prb / srslte_ra_type0_P(cell.nof_prb)))
              prbset_orig++;
            break;
          case LEFT_KEY:
            if (prbset_orig > 0)
              prbset_orig--;
            break;
          case UP_KEY:
            if (prbset_num < (int) ceilf((float) cell.nof_prb / srslte_ra_type0_P(cell.nof_prb)))
              prbset_num++;
            break;
          case DOWN_KEY:
            last_prbset_num = prbset_num;
            if (prbset_num > 0)
              prbset_num--;          
            break;          
        }
      } else {
        last_mcs_idx = mcs_idx; 
        mcs_idx = atoi(input);          
      }
      bzero(input,sizeof(input));
      if (update_radl()) {
        printf("Trying with last known MCS index\n");
        mcs_idx = last_mcs_idx; 
        prbset_num = last_prbset_num; 
        return update_radl();
      }
    }
    return 0; 
  } else if (n < 0) {
    // error
    perror("select");
    return -1; 
  } else {
    return 0; 
  }
}

#define DATA_BUFF_SZ    1024*128
uint8_t data[8*DATA_BUFF_SZ], data2[DATA_BUFF_SZ];
uint8_t data_tmp[DATA_BUFF_SZ]; 

uint8_t data_pcc[8*DATA_BUFF_SZ], data_tmp_pcc[DATA_BUFF_SZ]; 

/** Function run in a separate thread to receive UDP data */
void *net_thread_fnc(void *arg) {
  int n; 
  int rpm = 0, wpm=0; 
  
  do {
    n = srslte_netsource_read(&net_source, &data2[rpm], DATA_BUFF_SZ-rpm);
    if (n > 0) {
      int nbytes = 1+(pdsch_cfg.grant.mcs.tbs-1)/8;
      rpm += n; 
      INFO("received %d bytes. rpm=%d/%d\n",n,rpm,nbytes);
      wpm = 0; 
      while (rpm >= nbytes) {
        // wait for packet to be transmitted
        sem_wait(&net_sem);
        memcpy(data, &data2[wpm], nbytes);          
        INFO("Sent %d/%d bytes ready\n", nbytes, rpm);
        rpm -= nbytes;          
        wpm += nbytes; 
        net_packet_ready = true; 
      }
      if (wpm > 0) {
        INFO("%d bytes left in buffer for next packet\n", rpm);
        memcpy(data2, &data2[wpm], rpm * sizeof(uint8_t));
      }
    } else if (n == 0) {
      rpm = 0; 
    } else {
      fprintf(stderr, "Error receiving from network\n");
      exit(-1);
    }      
  } while(n >= 0);
  return NULL;
}

/** Function to receive PCC UDP data **/ 
void *net_thread_fnc_pcc(void *arg) {
  int n; 
  int rpm = 0, wpm=0; 
  
  do {
    n = srslte_netsource_read(&net_source_pcc, &data2[rpm], DATA_BUFF_SZ-rpm);
    if (n > 0) {
      int nbytes = 1+(pdsch_cfg.grant.mcs.tbs-1)/8;
      rpm += n; 
      INFO("received %d bytes. rpm=%d/%d\n",n,rpm,nbytes);
      wpm = 0; 
      while (rpm >= nbytes) {
        // wait for packet to be transmitted
        sem_wait(&net_sem_pcc);
        memcpy(data_pcc, &data2[wpm], nbytes);          
        INFO("Sent %d/%d bytes ready\n", nbytes, rpm);
        rpm -= nbytes;          
        wpm += nbytes; 
        net_packet_ready_pcc = true; 
      }
      if (wpm > 0) {
        INFO("%d bytes left in buffer for next packet\n", rpm);
        memcpy(data2, &data2[wpm], rpm * sizeof(uint8_t));
      }
    } else if (n == 0) {
      rpm = 0; 
    } else {
      fprintf(stderr, "Error receiving from network\n");
      exit(-1);
    }      
  } while(n >= 0);
  return NULL;
}

int main(int argc, char **argv) {
  int nf=0, sf_idx=0, N_id_2=0;
  cf_t pss_signal[SRSLTE_PSS_LEN];
  float sss_signal0[SRSLTE_SSS_LEN]; // for subframe 0
  float sss_signal5[SRSLTE_SSS_LEN]; // for subframe 5
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  int i;
  cf_t *sf_symbols[SRSLTE_MAX_PORTS];
  cf_t *slot1_symbols[SRSLTE_MAX_PORTS];

  cf_t *sf_symbols_pcc[SRSLTE_MAX_PORTS];
  cf_t *slot1_symbols_pcc[SRSLTE_MAX_PORTS];   

  srslte_dci_msg_t dci_msg;
  srslte_dci_location_t locations[SRSLTE_NSUBFRAMES_X_FRAME][30];
  uint32_t sfn; 
  srslte_chest_dl_t est; 
  
#ifdef DISABLE_RF
  if (argc < 3) {
    usage(argv[0]);
    exit(-1);
  }
#endif

  parse_args(argc, argv);

  N_id_2 = cell.id % 3;
  sf_n_re = 2 * SRSLTE_CP_NORM_NSYMB * cell.nof_prb * SRSLTE_NRE;
  sf_n_samples = 2 * SRSLTE_SLOT_LEN(srslte_symbol_sz(cell.nof_prb));

  cell.phich_length = SRSLTE_PHICH_NORM;
  cell.phich_resources = SRSLTE_PHICH_R_1;
  sfn = 0;

  prbset_num = (int) ceilf((float) cell.nof_prb / srslte_ra_type0_P(cell.nof_prb)); 
  last_prbset_num = prbset_num; 
  
  /* this *must* be called after setting slot_len_* */
  base_init();

  /* Generate PSS/SSS signals */
  srslte_pss_generate(pss_signal, N_id_2);
  srslte_sss_generate(sss_signal0, sss_signal5, cell.id);
  
  /* Generate CRS signals */
  if (srslte_chest_dl_init(&est, cell)) {
    fprintf(stderr, "Error initializing equalizer\n");
    exit(-1);
  }

  for (i = 0; i < SRSLTE_MAX_PORTS; i++) { // now there's only 1 port
    sf_symbols[i] = sf_buffer;
    slot1_symbols[i] = &sf_buffer[SRSLTE_SLOT_LEN_RE(cell.nof_prb, cell.cp)];
    sf_symbols_pcc[i] = sf_buffer_pcc;
    slot1_symbols_pcc[i] = &sf_buffer_pcc[SRSLTE_SLOT_LEN_RE(cell.nof_prb, cell.cp)];  
  }

#ifndef DISABLE_RF


  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  if (!output_file_name) {
    
    int srate = srslte_sampling_freq_hz(cell.nof_prb);    
    if (srate != -1) {  
      if (srate < 10e6) {          
        srslte_rf_set_master_clock_rate(&rf, 4*srate);
        srslte_rf_set_master_clock_rate(&rf_pcc, 4*srate);         
      } else {
        srslte_rf_set_master_clock_rate(&rf, srate);   
        srslte_rf_set_master_clock_rate(&rf_pcc,srate);     
      }
      printf("Setting sampling rate %.2f MHz\n", (float) srate/1000000);
      float srate_rf = srslte_rf_set_tx_srate(&rf, (double) srate);
      float srate_rf_pcc = srslte_rf_set_tx_srate(&rf_pcc,(double)srate);
      if (srate_rf != srate) {
        fprintf(stderr, "Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      fprintf(stderr, "Invalid number of PRB %d\n", cell.nof_prb);
      exit(-1);
    }
    printf("Set TX gain: %.1f dB\n", srslte_rf_set_tx_gain(&rf, rf_gain));
    printf("Set PCC TX gain: %.1f dB\n", srslte_rf_set_tx_gain(&rf_pcc, rf_gain));
    printf("Set TX freq: %.2f MHz\n",
        srslte_rf_set_tx_freq(&rf, rf_freq) / 1000000);
    printf("Set PCC TX freq: %.2f MHz\n",
        srslte_rf_set_tx_freq(&rf_pcc, rf_freq_pcc) / 1000000);

  }
#endif

  if (update_radl(sf_idx)) {
    exit(-1);
  }
  
  if (net_port > 0) {
    if (pthread_create(&net_thread, NULL, net_thread_fnc, NULL)) {
      perror("pthread_create");
      exit(-1);
    }
  }
 
  if (net_port_pcc > 0) {
    if (pthread_create(&net_thread_pcc, NULL, net_thread_fnc_pcc, NULL)) {
      perror("pthread_create");
      exit(-1);
    }
  }

  /* Initiate valid DCI locations */
  for (i=0;i<SRSLTE_NSUBFRAMES_X_FRAME;i++) {
    srslte_pdcch_ue_locations(&pdcch, locations[i], 30, i, cfi, UE_CRNTI);
    
  }
    
  nf = 0;
  
  bool send_data = false; 
  bool send_data_pcc = false; 

  srslte_softbuffer_tx_reset(&softbuffer);
  srslte_softbuffer_tx_reset(&softbuffer_pcc);

#ifndef DISABLE_RF
  bool start_of_burst = true; 
#endif
  
  while ((nf < nof_frames || nof_frames == -1) && !go_exit) {
    for (sf_idx = 0; sf_idx < SRSLTE_NSUBFRAMES_X_FRAME && (nf < nof_frames || nof_frames == -1); sf_idx++) {

      // --------------- Primary channel ------------------

      bzero(sf_buffer_pcc, sizeof(cf_t) * sf_n_re); 

      // Send PSS and SSS in both SF 0 and SF 5 of PCC; do not send any SS in SCC  
      if (sf_idx == 0 || sf_idx == 5) {
        srslte_pss_put_slot(pss_signal, sf_buffer_pcc, cell.nof_prb, SRSLTE_CP_NORM);
        srslte_sss_put_slot(sf_idx ? sss_signal5 : sss_signal0, sf_buffer_pcc, cell.nof_prb,
            SRSLTE_CP_NORM);
      }

      //Reference signal insertion for PCC 
      srslte_refsignal_cs_put_sf(cell, 0, est.csr_signal.pilots[0][sf_idx], sf_buffer_pcc); 

      // MIB only in SF 0 
      srslte_pbch_mib_pack(&cell, sfn, bch_payload);
      if (sf_idx == 0) {
        // srslte_pbch_encode(&pbch, bch_payload, slot1_symbols, nf%4);
        srslte_pbch_encode(&pbch, bch_payload, slot1_symbols_pcc, nf%4);
      }

      // PCFICH insertion for PCC (all SFs) 
      srslte_pcfich_encode(&pcfich, cfi, sf_symbols_pcc, sf_idx); 

      /* Transmit PDCCH + PDSCH only when there is data to send */
      if (net_port > 0) {
        // Data is available at input port to be sent
        send_data_pcc = net_packet_ready_pcc; 
        // Selectively disable the transmission on sec channel
        if (send_data_pcc) {
          INFO("Transmitting packet on PCC\n",0);
        }
      } else {
        // TODO: Modify this correctly
        INFO("SF: %d, Generating %d random bits\n", sf_idx, pdsch_cfg.grant.mcs.tbs);
        for (i=0;i<pdsch_cfg.grant.mcs.tbs/8;i++) {
          data[i] = rand()%256;
        }
        /* Uncomment this to transmit on sf 0 and 5 only  */
        if (sf_idx != 0 && sf_idx != 5) {
          send_data = true; 
        } else {
          send_data = false;           
        }
      }        

      // ---------------- Secondary channel -------------------
      bzero(sf_buffer, sizeof(cf_t) * sf_n_re); 

      // TODO: Active subcarrier should probably come from parameters
      // Reference signal insertion  
      if (sf_idx >= sf_start && sf_idx <= sf_end) {
        srslte_refsignal_cs_put_sf(cell, 0, est.csr_signal.pilots[0][sf_idx], sf_buffer); 
      }

      // PCFICH insertion for specific SFs
      if (sf_idx >= sf_start || sf_idx <= sf_end) {
        srslte_pcfich_encode(&pcfich, cfi, sf_symbols, sf_idx); 
      }        

      /* Update DL resource allocation from control port */
      if (update_control(sf_idx)) {
        fprintf(stderr, "Error updating parameters from control port\n");
      }

      /* Transmit PDCCH + PDSCH only when there is data to send */
      if (net_port > 0) {
        // Data is available at input port to be sent
        send_data = net_packet_ready; 
        // Selectively disable the transmission on sec channel
        if (sf_idx < sf_start || sf_idx > sf_end) { 
          send_data = false;
        }
        if (send_data) {
          INFO("Transmitting packet on SCC\n",0);
        }
      } else {
        // TODO: Modify this correctly
        INFO("SF: %d, Generating %d random bits\n", sf_idx, pdsch_cfg.grant.mcs.tbs);
        for (i=0;i<pdsch_cfg.grant.mcs.tbs/8;i++) {
          data[i] = rand()%256;
        }
        /* Uncomment this to transmit on sf 0 and 5 only  */
        if (sf_idx != 0 && sf_idx != 5) {
          send_data = true; 
        } else {
          send_data = false;           
        }
      }        

      // ------------ Sending data in PCC --------------
      if (send_data_pcc) {

        /* Encode PDCCH */
        INFO("Putting DCI to location: n=%d, L=%d\n", locations[sf_idx][0].ncce, locations[sf_idx][0].L);
        srslte_dci_msg_pack_pdsch(&ra_dl, SRSLTE_DCI_FORMAT1, &dci_msg, cell.nof_prb, false);
        if (srslte_pdcch_encode(&pdcch, &dci_msg, locations[sf_idx][0], UE_CRNTI, sf_symbols_pcc, sf_idx, cfi)) {
          fprintf(stderr, "Error encoding DCI message\n");
          exit(-1);
        }

        // FIXME: The configuration parameters seem to be same as SCC
        /* Configure pdsch_cfg parameters */
        srslte_ra_dl_grant_t grant; 
        srslte_ra_dl_dci_to_grant(&ra_dl, cell.nof_prb, UE_CRNTI, &grant);        
        if (srslte_pdsch_cfg(&pdsch_cfg, cell, &grant, cfi, sf_idx, 0)) {
          fprintf(stderr, "Error configuring PDSCH\n");
          exit(-1);
        }

        /* Encode PDSCH */
        if (srslte_pdsch_encode(&pdsch, &pdsch_cfg, &softbuffer_pcc, data_pcc, sf_symbols)) {
          fprintf(stderr, "Error encoding PDSCH\n");
          exit(-1);
        }        
        if (net_port_pcc > 0 && net_packet_ready_pcc) {
          if (null_file_sink) {
            srslte_bit_pack_vector(data_pcc, data_tmp_pcc, pdsch_cfg.grant.mcs.tbs);
            if (srslte_netsink_write(&net_sink_pcc, data_tmp_pcc, 1+(pdsch_cfg.grant.mcs.tbs-1)/8) < 0) {
              fprintf(stderr, "Error sending data through UDP socket\n");
            }            
          }
          net_packet_ready_pcc = false; 
          sem_post(&net_sem_pcc);
        }
      } 
      /* Transform to OFDM symbols */
      srslte_ofdm_tx_sf(&ifft, sf_buffer_pcc, output_buffer_pcc);

      // ------------ Sending data in SCC --------------
      if (send_data) {

        /* Encode PDCCH */
        INFO("Putting DCI to location: n=%d, L=%d\n", locations[sf_idx][0].ncce, locations[sf_idx][0].L);
        srslte_dci_msg_pack_pdsch(&ra_dl, SRSLTE_DCI_FORMAT1, &dci_msg, cell.nof_prb, false);
        if (srslte_pdcch_encode(&pdcch, &dci_msg, locations[sf_idx][0], UE_CRNTI, sf_symbols, sf_idx, cfi)) {
          fprintf(stderr, "Error encoding DCI message\n");
          exit(-1);
        }

        /* Configure pdsch_cfg parameters */
        srslte_ra_dl_grant_t grant; 
        srslte_ra_dl_dci_to_grant(&ra_dl, cell.nof_prb, UE_CRNTI, &grant);        
        if (srslte_pdsch_cfg(&pdsch_cfg, cell, &grant, cfi, sf_idx, 0)) {
          fprintf(stderr, "Error configuring PDSCH\n");
          exit(-1);
        }

        /* Encode PDSCH */
        if (srslte_pdsch_encode(&pdsch, &pdsch_cfg, &softbuffer, data, sf_symbols)) {
          fprintf(stderr, "Error encoding PDSCH\n");
          exit(-1);
        }        
        if (net_port > 0 && net_packet_ready) {
          if (null_file_sink) {
            srslte_bit_pack_vector(data, data_tmp, pdsch_cfg.grant.mcs.tbs);
            if (srslte_netsink_write(&net_sink, data_tmp, 1+(pdsch_cfg.grant.mcs.tbs-1)/8) < 0) {
              fprintf(stderr, "Error sending data through UDP socket\n");
            }            
          }
          net_packet_ready = false; 
          sem_post(&net_sem);
        }
      } 

      /* Transform to OFDM symbols */
      srslte_ofdm_tx_sf(&ifft, sf_buffer, output_buffer);

      /* send to file or usrp */
      if (output_file_name) {
        if (!null_file_sink) {
          srslte_filesink_write(&fsink, output_buffer, sf_n_samples);          
        }
        usleep(1000);
      } else {
#ifndef DISABLE_RF
        // FIXME
        float norm_factor = (float) cell.nof_prb/15/sqrtf(pdsch_cfg.grant.nof_prb);

        srslte_vec_sc_prod_cfc(output_buffer_pcc, rf_amp_pcc*norm_factor, output_buffer_pcc, SRSLTE_SF_LEN_PRB(cell.nof_prb));
        srslte_rf_send2(&rf_pcc, output_buffer_pcc, sf_n_samples, true, start_of_burst, false);

        srslte_vec_sc_prod_cfc(output_buffer, rf_amp*norm_factor, output_buffer, SRSLTE_SF_LEN_PRB(cell.nof_prb));
        srslte_rf_send2(&rf, output_buffer, sf_n_samples, true, start_of_burst, false);

        start_of_burst=false; 
#endif
      }
    }
    nf++;
    sfn = (sfn + 1) % 1024;
  }

  base_free();

  printf("Done\n");
  exit(0);
}


