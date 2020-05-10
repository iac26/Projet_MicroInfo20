Pour que notre projet fonctionne correctement, il faut remplacer le fichier 
spi_comm.c de e-puck2_main-processor par notre fichier spi-comm.c modifié.

On a modifié la ligne 102 de cette manière après avoir discuté avec un assistant:
if(DCMID.state == DCMI_ACTIVE_STREAM || DCMID.state == DCMI_ACTIVE_ONESHOT || DCMID.state == DCMI_READY){

