

void Add_To_Buffer(uint32_t data,buff_type* buffer) {
	buffer->data[buffer->head++%(sizeof(buffer->data)/4)]=data
}

uint32_t Get_From_Buffer() {

}
