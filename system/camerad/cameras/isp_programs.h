unsigned char isp_prog1[] = "\x05\x00\x00\x04\x2C\x00\x00\x00\xFF\xFF\xFF\xFF\x30\x00\x00\x00\xFF\xFF\xFF\xFF\x34\x00\x00\x00\xFF\xFF\xFF\xFF\x38\x00\x00\x00\xFF\xFF\xFF\xFF\x3C\x00\x00\x00\xFF\xFF\xFF\xFF\x21\x00\x00\x03\xDC\x04\x00\x00\x00\x00\x00\x00\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x1F\x01\x21\x0B\x00\x00\x50\x07\x24\x0C\x00\x09\x03\x00\x00\x03\xB0\x06\x00\x00\xF3\x03\x43\x04\x00\x00\x00\x00\x00\x00\x00\x00\x07\x00\x00\x03\x60\x05\x00\x00\x31\x3F\x30\x3F\x01\x04\x01\x04\x01\x04\x01\x04\x01\x04\x01\x04\x01\x04\x01\x04\xCA\x00\x00\x00\x9C\x00\x00\x00\x04\x00\x00\x03\xFC\x06\x00\x00\x80\x00\xBF\x00\x06\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xF8\x06\x00\x00\x00\x01\x00\x00\x02\x00\x00\x03\x1C\x07\x00\x00\x00\x80\x00\x00\x66\x00\x00\x08\x0D\x00\x00\x03\x60\x07\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x94\x07\x00\x00\x00\x00\x00\x00\xFF\x01\x21\x0B\x20\x01\x50\x07\x24\x0C\x00\x18\x01\x00\x00\x03\x98\x07\x00\x00\x00\x00\x00\x00\xFF\x00\x21\x0A\xD0\x11\x52\x07\x24\x0C\x00\x1A\xFF\x00\x21\x0A\xD0\x12\x52\x07\x24\x0C\x00\x1C\xFF\x00\x21\x0A\xD0\x13\x52\x07\x24\x0C\x00\x1E\x0C\x00\x00\x03\x30\x0F\x00\x00\x59\x02\x75\x00\x32\x01\x00\x00\x00\x00\x00\x00\x00\x00\xFF\x03\xAE\x1E\xFE\x01\x54\x1F\x00\x00\x00\x00\x00\x02\x00\x00\xFF\x03\x55\x1E\xAD\x1F\xFE\x01\x00\x00\x00\x00\x00\x02\x00\x00\xFF\x03";

unsigned char isp_prog2[] = "\x05\x00\x00\x04\x2C\x00\x00\x00\xFF\xFF\xFF\xFF\x30\x00\x00\x00\xFF\xFF\xFF\xFF\x34\x00\x00\x00\xFF\xFF\xFF\xFF\x38\x00\x00\x00\xFF\xFF\xFF\xFF\x3C\x00\x00\x00\xFF\xFF\xFF\xFF\x21\x00\x00\x03\xDC\x04\x00\x00\x00\x00\x00\x00\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x0C\x08\x0E\x00\x08\x18\x0A\x10\x04\x28\x06\x20\x00\x38\x02\x30\x0A\x08\x0C\x00\x07\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x09\x08\x0B\x00\x06\x18\x08\x10\x03\x28\x05\x20\x00\x38\x02\x30\x0C\x08\x0E\x00\x08\x18\x0A\x10\x04\x28\x06\x20\x00\x38\x02\x30\x1F\x01\x21\x0B\x00\x00\x50\x07\x24\x0C\x00\x09\x03\x00\x00\x03\xB0\x06\x00\x00\xF3\x03\x43\x04\x00\x00\x00\x00\x00\x00\x00\x00\x07\x00\x00\x03\x60\x05\x00\x00\x31\x3F\x30\x3F\x93\x06\x93\x06\x93\x06\x93\x06\x93\x06\x93\x06\x93\x06\x93\x06\xCA\x00\x00\x00\x9C\x00\x00\x00\x08\x00\x00\x03\xC4\x05\x00\x00\x60\x7E\x00\x00\xE7\x1C\x00\x00\x37\x19\x00\x00\xDB\x08\x00\x00\x27\x0A\x00\x00\x2B\x00\x2B\x00\x40\x20\x80\x00\x00\x00\x00\x00\x04\x00\x00\x03\xFC\x06\x00\x00\x80\x00\xCA\x00\xE7\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xF8\x06\x00\x00\x00\x01\x00\x00\x02\x00\x00\x03\x1C\x07\x00\x00\x00\x80\x00\x00\x66\x00\x00\x08\x0D\x00\x00\x03\x60\x07\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x94\x07\x00\x00\x01\x00\x00\x00\xFF\x01\x21\x0B\x20\x01\x50\x07\x24\x0C\x00\x19\x01\x00\x00\x03\x98\x07\x00\x00\x00\x00\x00\x00\xFF\x00\x21\x0A\xD0\x11\x52\x07\x24\x0C\x00\x1A\xFF\x00\x21\x0A\xD0\x12\x52\x07\x24\x0C\x00\x1C\xFF\x00\x21\x0A\xD0\x13\x52\x07\x24\x0C\x00\x1E";

unsigned char meta_right[] = "\x01\x00\x00\x03\xE8\x05\x00\x00\x0D\x30\x36\x06\x16\x00\x00\x03\xF4\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x38\x18\x3A\x40\xE6\x60\x00\x00\x00\xFF\x00\x28\x03\xB6\x07\xFF\x80\x0F\x00\xFF\xED\x0F\x00\xFF\xE0\x0F\x00\xFF\xF7\x0F\x00\xFF\x80\x0F\x00\xFF\xED\x0F\x00\xFF\xE0\x0F\x00\xFF\xF7\x0F\x00\x0A\x00\x00\x00\x08\x08\x21\x00\x33\xF3\x03\x00\xA0\x00\x01\x00\x03\x00\x00\x0A\x03\x00\x00\x0A\xFF\x00\x21\x0A\x00\x03\x52\x07\x24\x0C\x00\x0C\x0B\x00\x00\x03\xBC\x06\x00\x00\x00\x00\x3D\x0B\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x07\x00\x00\xC0\x06\xC0\x00\x03\x00\x00\x00\xC0\x00\xC0\x00\x01\x73\x03\x21\x0A\xE8\x0A\x52\x07\x24\x0C\x00\x12\x73\x03\x21\x0A\x5C\x0E\x52\x07\x24\x0C\x00\x13\x0B\x00\x00\x03\x3C\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x0A\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\x73\x08\x01\x80\xCF\x0B\xDF\x07\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\x68\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x05\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\x73\x08\x01\x80\xCF\x0B\xEF\x03\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x10\x0E\x00\x00\xE3\x06\xFC\x00\xA1\x05\x62\x00\x02\x00\x00\x03\x30\x0E\x00\x00\x71\x03\x7E\x00\xA1\x05\x62\x00\x0B\x00\x00\x03\xAC\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x02\xCC\x4C\x06\x20\x80\x01\x00\x00\x88\x9A\x00\x00\x73\x08\x09\x80\xCF\x0B\xDF\x01\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\xD8\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x01\xCC\x4C\x06\x10\xC0\x00\x00\x00\x44\x4D\x00\x00\x73\x08\x09\x80\xCF\x0B\xEF\x00\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x50\x0E\x00\x00\xA1\x01\x3A\x00\x55\x01\x16\x00\x02\x00\x00\x03\x70\x0E\x00\x00\xD0\x00\x1D\x00\x55\x01\x16\x00\x02\x00\x00\x03\x84\x0D\x00\x00\xE3\x06\xFC\x00\xA1\x05\x02\x00\x02\x00\x00\x03\xA4\x0D\x00\x00\x71\x03\x7E\x00\xA1\x05\x02\x00\x02\x00\x00\x03\x90\x0D\x00\x00\x79\x01\x00\x00\x67\x01\x18\x00\x02\x00\x00\x03\xB0\x0D\x00\x00\xBC\x00\x00\x00\x67\x01\x18\x00\x01\x00\x00\x03\x6C\x0D\x00\x00\x00\x03\x00\x00\x02\x00\x00\x03\xC4\x0D\x00\x00\x79\x01\x00\x00\x67\x01\x00\x00\x02\x00\x00\x03\xE4\x0D\x00\x00\xBC\x00\x00\x00\x67\x01\x00\x00\x02\x00\x00\x03\xD0\x0D\x00\x00\x5F\x00\x00\x00\x59\x00\x0A\x00\x02\x00\x00\x03\xF0\x0D\x00\x00\x2F\x00\x00\x00\x59\x00\x0A\x00\x01\x00\x00\x03\x7C\x0D\x00\x00\x00\x0F\x00\x00\x07\x00\x00\x03\xB8\x0A\x00\x00\x50\x01\x00\x00\x1C\x00\x2F\x00\x3D\x00\x3D\x00\xC0\x3B\xC0\x3B\xC0\x3B\xC0\x3B\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x0C\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x98\x0C\x00\x00\x00\xFF\xFF\x00\x07\x00\x00\x03\xC8\x0B\x00\x00\x50\x01\x00\x00\x1C\x00\x2F\x00\x3D\x00\x3D\x00\xC0\x3B\xC0\x3B\xC0\x3B\xC0\x3B\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x14\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xB0\x0C\x00\x00\x01\xFF\xFF\x00\x02\x00\x00\x03\xD4\x0A\x00\x00\x30\x01\x00\x00\xA1\x03\xE7\x05\x02\x00\x00\x03\xE4\x0B\x00\x00\x30\x01\x00\x00\xA1\x03\xE7\x05\x07\x00\x00\x03\xAC\x0B\x00\x00\x12\x01\x00\x00\x0E\x00\x17\x00\x7D\x00\x7D\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x10\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xAC\x0C\x00\x00\x00\xFF\xFF\x00\x03\x00\x00\x03\xE4\x0A\x00\x00\x06\x22\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\x00\x00\x03\xF0\x0A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xD9\x07\x00\x00\x27\xF8\x75\xEA\x4F\x57\xB1\xCF\x8B\x15\x00\x00\xD7\x37\x6C\xD4\x03\x00\x00\x03\x40\x0B\x00\x00\xD9\x07\x00\x00\x27\xF8\x75\xEA\x4F\x57\xB1\xCF\x02\x00\x00\x03\x9C\x0C\x00\x00\x8B\x15\x00\x00\xD7\x37\x6C\xD4\x02\x00\x00\x03\xA4\x0C\x00\x00\x40\x00\x20\x00\x34\x08\xB0\x0B\x01\x00\x00\x03\x4C\x0B\x00\x00\x03\x00\x03\x00\x05\x00\x00\x03\x50\x0B\x00\x00\xB0\x04\x00\x00\x00\x10\x08\x04\x46\xC2\x40\x10\x10\x04\x41\x10\x10\x04\x00\x00\x05\x00\x00\x03\x78\x0B\x00\x00\xB0\x04\x00\x00\x00\x10\x08\x04\x46\xC2\x40\x10\x10\x04\x41\x10\x10\x04\x00\x00\x02\x00\x00\x03\x8C\x0B\x00\x00\x10\x00\x00\x00\x10\x00\x00\x00\x06\x00\x00\x03\x94\x0B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x17\x00\x21\x0B\xD0\x08\x50\x07\x24\x0C\x00\x41\x7F\x00\x21\x0A\x50\x15\x52\x07\x24\x0C\x00\x43\x03\x00\x00\x03\xEC\x0B\x00\x00\x94\x00\x00\x00\x03\x00\xEF\x03\xFB\x00\x02\x00\x01\x00\x00\x03\x40\x00\x00\x00\xC6\x0D\x00\x00\x01\x00\x00\x03\x48\x00\x00\x00\x0E\x00\x00\x00\x01\x00\x00\x03\x4C\x00\x00\x00\xFF\xFD\x00\x00\x01\x00\x00\x03\x4C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x6C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x0C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x2C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x8C\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xAC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xCC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xEC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x44\x00\x00\x00\x3F\x01\x00\x00\x01\x00\x00\x03\xAC\x0A\x00\x00\x58\x0D\x00\x00\x01\x00\x00\x03\x00\x0F\x00\x00\x00\x00\x00\x00";
unsigned char meta_left[] = "\x01\x00\x00\x03\xE8\x05\x00\x00\x0D\x30\x36\x06\x16\x00\x00\x03\xF4\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x38\x18\x3A\x40\xE6\x60\x00\x00\x00\xFF\x00\x28\x03\xB6\x07\xFF\x80\x0F\x00\xFF\xED\x0F\x00\xFF\xE0\x0F\x00\xFF\xF7\x0F\x00\xFF\x80\x0F\x00\xFF\xED\x0F\x00\xFF\xE0\x0F\x00\xFF\xF7\x0F\x00\x0A\x00\x00\x00\x08\x08\x21\x00\x33\xF3\x03\x00\xA0\x00\x01\x00\x03\x00\x00\x0A\x03\x00\x00\x0A\xFF\x00\x21\x0A\x00\x02\x52\x07\x24\x0C\x00\x0D\x0B\x00\x00\x03\xBC\x06\x00\x00\x00\x00\x3D\x0B\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x00\x00\x00\xC0\x00\xC0\x00\x01\x00\x00\x00\xC0\x00\xC0\x00\x01\x73\x03\x21\x0A\x00\x04\x52\x07\x24\x0C\x00\x12\x73\x03\x21\x0A\x74\x07\x52\x07\x24\x0C\x00\x13\x0B\x00\x00\x03\x3C\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x0A\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xE7\x08\x00\x80\xCF\x0B\xDF\x07\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\x68\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x05\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xE7\x08\x00\x80\xCF\x0B\xEF\x03\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x10\x0E\x00\x00\xE3\x06\xFC\x00\x3F\x05\x00\x00\x02\x00\x00\x03\x30\x0E\x00\x00\x71\x03\x7E\x00\x3F\x05\x00\x00\x0B\x00\x00\x03\xAC\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x02\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xDB\x08\x00\x80\xCF\x0B\xDF\x01\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\xD8\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x01\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xDB\x08\x00\x80\xCF\x0B\xEF\x00\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x50\x0E\x00\x00\xA1\x01\x3A\x00\x3F\x01\x00\x00\x02\x00\x00\x03\x70\x0E\x00\x00\xD0\x00\x1D\x00\x3F\x01\x00\x00\x02\x00\x00\x03\x84\x0D\x00\x00\xE3\x06\xFC\x00\xEF\x05\x00\x00\x02\x00\x00\x03\xA4\x0D\x00\x00\x71\x03\x7E\x00\xEF\x05\x00\x00\x02\x00\x00\x03\x90\x0D\x00\x00\x79\x01\x00\x00\x4F\x01\x00\x00\x02\x00\x00\x03\xB0\x0D\x00\x00\xBC\x00\x00\x00\x4F\x01\x00\x00\x01\x00\x00\x03\x6C\x0D\x00\x00\x00\x03\x00\x00\x02\x00\x00\x03\xC4\x0D\x00\x00\x79\x01\x00\x00\x7B\x01\x00\x00\x02\x00\x00\x03\xE4\x0D\x00\x00\xBC\x00\x00\x00\x7B\x01\x00\x00\x02\x00\x00\x03\xD0\x0D\x00\x00\x5F\x00\x00\x00\x57\x00\x00\x00\x02\x00\x00\x03\xF0\x0D\x00\x00\x2F\x00\x00\x00\x57\x00\x00\x00\x01\x00\x00\x03\x7C\x0D\x00\x00\x00\x0F\x00\x00\x07\x00\x00\x03\xB8\x0A\x00\x00\x00\x00\x00\x00\x22\x00\x2F\x00\x3D\x00\x3D\x00\xC0\x3B\xC0\x3B\xC0\x3B\xC0\x3B\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x0C\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x98\x0C\x00\x00\x00\xFF\xFF\x00\x07\x00\x00\x03\xC8\x0B\x00\x00\x00\x00\x00\x00\x22\x00\x2F\x00\x3D\x00\x3D\x00\xC0\x3B\xC0\x3B\xC0\x3B\xC0\x3B\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x14\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xB0\x0C\x00\x00\x01\xFF\xFF\x00\x02\x00\x00\x03\xD4\x0A\x00\x00\x00\x00\x00\x00\x3D\x04\xE7\x05\x02\x00\x00\x03\xE4\x0B\x00\x00\x00\x00\x00\x00\x3D\x04\xE7\x05\x07\x00\x00\x03\xAC\x0B\x00\x00\x00\x00\x00\x00\x10\x00\x17\x00\x7D\x00\x7D\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x10\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xAC\x0C\x00\x00\x00\xFF\xFF\x00\x03\x00\x00\x03\xE4\x0A\x00\x00\x06\x22\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\x00\x00\x03\xF0\x0A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xD9\x07\x00\x00\x27\xF8\x75\xEA\x4F\x57\xB1\xCF\x8B\x15\x00\x00\xD7\x37\x6C\xD4\x03\x00\x00\x03\x40\x0B\x00\x00\xD9\x07\x00\x00\x27\xF8\x75\xEA\x4F\x57\xB1\xCF\x02\x00\x00\x03\x9C\x0C\x00\x00\x8B\x15\x00\x00\xD7\x37\x6C\xD4\x02\x00\x00\x03\xA4\x0C\x00\x00\x40\x00\x20\x00\xA8\x08\xB0\x0B\x01\x00\x00\x03\x4C\x0B\x00\x00\x03\x00\x03\x00\x05\x00\x00\x03\x50\x0B\x00\x00\xB0\x04\x00\x00\x00\x10\x08\x04\x46\xC2\x40\x10\x10\x04\x41\x10\x10\x04\x00\x00\x05\x00\x00\x03\x78\x0B\x00\x00\xB0\x04\x00\x00\x00\x10\x08\x04\x46\xC2\x40\x10\x10\x04\x41\x10\x10\x04\x00\x00\x02\x00\x00\x03\x8C\x0B\x00\x00\x10\x00\x00\x00\x10\x00\x00\x00\x06\x00\x00\x03\x94\x0B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x17\x00\x21\x0B\x20\x03\x50\x07\x24\x0C\x00\x41\x7F\x00\x21\x0A\xD0\x14\x52\x07\x24\x0C\x00\x43\x03\x00\x00\x03\xEC\x0B\x00\x00\x00\x00\x00\x00\x03\x00\xEF\x03\xFB\x00\x02\x00\x01\x00\x00\x03\x40\x00\x00\x00\xC6\x0D\x00\x00\x01\x00\x00\x03\x48\x00\x00\x00\x0E\x00\x00\x00\x01\x00\x00\x03\x4C\x00\x00\x00\xFF\xFD\x00\x00\x01\x00\x00\x03\x4C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x6C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x0C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x2C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x8C\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xAC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xCC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xEC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x44\x00\x00\x00\x3F\x01\x00\x00\x01\x00\x00\x03\xAC\x0A\x00\x00\x58\x0D\x00\x00\x01\x00\x00\x03\x00\x0F\x00\x00\x00\x00\x00\x00";
unsigned char meta_dual[] = "\x13\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x00\x30\x00\x00\x00\x00\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x00\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x01\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x60\x01\x00\x00\x00\x00\x00\x00\x02\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x01\x00\x00\x00\x00\x00\x00\x04\x30\x00\x00\x00\x00\x00\x00\x40\x01\x00\x00\x00\x00\x00\x00\x04\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3B\x01\x00\x00\x00\x00\x00\x0A\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0C\x00\x00\x00\x00\x00\x00\x0B\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x02\x00\x00\x00\x00\x00\x0C\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x00\x0D\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x3B\x01\x00\x00\x00\x00\x00\x0E\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x0F\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x00\x00\x00\x00\x00\x10\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x05\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x00\x30\x00\x00\x40\x05\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x00\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x05\x00\x00\x40\x05\x00\x00\x00\x00\x00\x00\x01\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x60\x01\x00\x00\x40\x01\x00\x00\x00\x00\x00\x00\x02\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x01\x00\x00\x40\x01\x00\x00\x00\x00\x00\x00\x04\x30\x00\x00\x40\x01\x00\x00\x40\x01\x00\x00\x00\x00\x00\x00\x04\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x02\x00\x00\x05\x01\x00\x00\x00\x00\x00\x0A\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0C\x00\x00\x00\x0C\x00\x00\x00\x00\x00\x00\x0B\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x02\x00\x00\x40\x02\x00\x00\x00\x00\x00\x0C\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x88\x16\x00\x00\x40\x00\x00\x00\x00\x00\x00\x00\x0D\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x8C\x0A\x00\x00\x05\x01\x00\x00\x00\x00\x00\x0E\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x40\x00\x00\x00\x40\x00\x00\x00\x00\x00\x00\x0F\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00\x00\x00\x80\x00\x00\x00\x00\x00\x00\x10\x30\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

unsigned char meta_right_0[] = "\x02\x00\x00\x03\x78\x04\x00\x00\x04\x00\x00\x00\xC0\x00\x40\x00\x03\x00\x00\x03\x88\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0F\x0F\x00\x00\x02\x00\x00\x03\x9C\x04\x00\x00\x01\x00\x00\x00\xFF\x3F\x13\x00\x02\x00\x00\x03\xE4\x0C\x00\x00\xBF\x0F\x4C\x07\xCF\x0B\x00\x00\x01\x00\x00\x03\xE8\x05\x00\x00\x05\x30\x36\x06\x16\x00\x00\x03\xF4\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x38\x18\x3A\x40\xE6\x60\x00\x00\x00\x8F\x02\xC2\x05\x3C\x0A\xFF\xC8\x0F\x00\xFF\xA0\x0F\x00\xFF\xE4\x0F\x00\xFF\xB1\x0F\x00\xFF\xC8\x0F\x00\xFF\xA0\x0F\x00\xFF\xE4\x0F\x00\xFF\xB1\x0F\x00\x0A\x00\x00\x00\x10\x10\x40\x00\x44\xF3\x03\x00\xA0\x00\x40\x00\x66\xE0\x0C\x07\x66\xE0\x0C\x07\xFF\x00\x21\x0A\x00\x03\x52\x07\x24\x0C\x00\x0C\x0B\x00\x00\x03\xBC\x06\x00\x00\x00\x00\x3C\x0B\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x07\x00\x00\xC0\x06\xC0\x00\x03\x00\x00\x00\xC0\x00\xC0\x00\x01\x73\x03\x21\x0A\xE8\x0A\x52\x07\x24\x0C\x00\x0E\x73\x03\x21\x0A\x5C\x0E\x52\x07\x24\x0C\x00\x0F\x0B\x00\x00\x03\x3C\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x0A\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\x73\x08\x01\x80\xCF\x0B\xDF\x07\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\x68\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x05\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\x73\x08\x01\x80\xCF\x0B\xEF\x03\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x10\x0E\x00\x00\xE3\x06\xFC\x00\xA1\x05\x62\x00\x02\x00\x00\x03\x30\x0E\x00\x00\x71\x03\x7E\x00\xA1\x05\x62\x00\x02\x00\x00\x03\x18\x0E\x00\x00\x00\x00\xF0\x0F\x16\x00\x00\x00\x02\x00\x00\x03\x38\x0E\x00\x00\x00\x00\xF0\x0F\x17\x00\x00\x00\x0B\x00\x00\x03\xAC\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x02\xCC\x4C\x06\x20\x80\x01\x00\x00\x88\x9A\x00\x00\x73\x08\x09\x80\xCF\x0B\xDF\x01\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\xD8\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x01\xCC\x4C\x06\x10\xC0\x00\x00\x00\x44\x4D\x00\x00\x73\x08\x09\x80\xCF\x0B\xEF\x00\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x50\x0E\x00\x00\xA1\x01\x3A\x00\x55\x01\x16\x00\x02\x00\x00\x03\x70\x0E\x00\x00\xD0\x00\x1D\x00\x55\x01\x16\x00\x02\x00\x00\x03\x58\x0E\x00\x00\x00\x00\xF0\x0F\x16\x00\x00\x00\x02\x00\x00\x03\x78\x0E\x00\x00\x00\x00\xF0\x0F\x17\x00\x00\x00\x02\x00\x00\x03\x84\x0D\x00\x00\xE3\x06\xFC\x00\xA1\x05\x02\x00\x02\x00\x00\x03\xA4\x0D\x00\x00\x71\x03\x7E\x00\xA1\x05\x02\x00\x03\x00\x00\x03\x60\x0D\x00\x00\x00\x03\xE8\x05\x7D\x6C\x01\x09\x00\x03\xF4\x02\x02\x00\x00\x03\x90\x0D\x00\x00\x79\x01\x00\x00\x67\x01\x18\x00\x02\x00\x00\x03\xB0\x0D\x00\x00\xBC\x00\x00\x00\x67\x01\x18\x00\x02\x00\x00\x03\x98\x0D\x00\x00\x00\x00\xF0\x3F\x06\x00\x00\x00\x02\x00\x00\x03\xB8\x0D\x00\x00\x00\x00\xF0\x3F\x07\x00\x00\x00\x01\x00\x00\x03\x6C\x0D\x00\x00\x00\x03\x00\x00\x02\x00\x00\x03\xC4\x0D\x00\x00\x79\x01\x00\x00\x67\x01\x00\x00\x02\x00\x00\x03\xE4\x0D\x00\x00\xBC\x00\x00\x00\x67\x01\x00\x00\x03\x00\x00\x03\x70\x0D\x00\x00\x00\x0F\x7A\x01\x7D\x6C\x01\x09\x00\x0F\xBD\x00\x02\x00\x00\x03\xD0\x0D\x00\x00\x5F\x00\x00\x00\x59\x00\x0A\x00\x02\x00\x00\x03\xF0\x0D\x00\x00\x2F\x00\x00\x00\x59\x00\x0A\x00\x02\x00\x00\x03\xD8\x0D\x00\x00\x00\x00\xF0\x3F\x06\x00\x00\x00\x02\x00\x00\x03\xF8\x0D\x00\x00\x00\x00\xF0\x3F\x07\x00\x00\x00\x01\x00\x00\x03\x7C\x0D\x00\x00\x00\x0F\x00\x00\x07\x00\x00\x03\xB8\x0A\x00\x00\x74\x01\x00\x00\x17\x00\x2F\x00\x37\x00\x37\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x0C\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x98\x0C\x00\x00\x00\xFF\xFF\x00\x07\x00\x00\x03\xC8\x0B\x00\x00\x74\x01\x00\x00\x17\x00\x2F\x00\x37\x00\x37\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x14\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xB0\x0C\x00\x00\x00\xFF\xFF\x00\x02\x00\x00\x03\xD4\x0A\x00\x00\x30\x01\x00\x00\xA1\x03\xE7\x05\x02\x00\x00\x03\xE4\x0B\x00\x00\x30\x01\x00\x00\xD7\x02\x50\x05\x07\x00\x00\x03\xAC\x0B\x00\x00\x12\x01\x00\x00\x0E\x00\x17\x00\x7D\x00\x7D\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x10\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xAC\x0C\x00\x00\x00\xFF\xFF\x00\x03\x00\x00\x03\xE4\x0A\x00\x00\x00\x23\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\x00\x00\x03\xF0\x0A\x00\x00\x3F\x3E\x3F\x01\x05\x08\x0A\x08\x05\x01\x3F\x3E\x3F\x00\x00\x00\xE9\x05\x00\x00\x17\xFA\xC5\xF8\x94\x6D\xD2\xCB\x3B\x07\x00\x00\xA9\x77\x7B\xC6\x03\x00\x00\x03\x40\x0B\x00\x00\x46\x39\x76\x8D\x46\x39\xC5\xF8\xD5\x71\xC0\xCC\x02\x00\x00\x03\x9C\x0C\x00\x00\x3B\x07\x00\x00\xA9\x77\x7B\xC6\x02\x00\x00\x03\xA4\x0C\x00\x00\x40\x00\x20\x00\x34\x08\xB0\x0B\x01\x00\x00\x03\x4C\x0B\x00\x00\x0D\x00\x00\x00\x05\x00\x00\x03\x50\x0B\x00\x00\x00\x00\x01\x00\x00\x04\x41\x10\x10\x04\x41\x10\x10\x04\x41\x10\x10\x04\x00\x00\x05\x00\x00\x03\x78\x0B\x00\x00\x00\x00\x01\x00\x00\x04\x41\x10\x10\x04\x41\x10\x10\x04\x41\x10\x10\x04\x00\x00\x02\x00\x00\x03\x8C\x0B\x00\x00\x10\x00\x00\x00\x10\x00\x00\x00\x06\x00\x00\x03\x94\x0B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x87\x00\x21\x0B\xD0\x08\x50\x07\x24\x0C\x00\x40\x7F\x00\x21\x0A\x50\x15\x52\x07\x24\x0C\x00\x42\x03\x00\x00\x03\xEC\x0B\x00\x00\x94\x00\x00\x00\x07\x00\xEF\x03\xFB\x00\x02\x00\x01\x00\x00\x03\x40\x00\x00\x00\x86\x0D\x00\x00\x01\x00\x00\x03\x48\x00\x00\x00\x0E\x00\x00\x00\x01\x00\x00\x03\x4C\x00\x00\x00\xFF\xFD\x00\x00\x01\x00\x00\x03\x4C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x6C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x0C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x2C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x8C\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xAC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xCC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xEC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x44\x00\x00\x00\x3F\x01\x00\x00\x01\x00\x00\x03\xAC\x0A\x00\x00\x48\x0D\x10\x00\x01\x00\x00\x03\x00\x0F\x00\x00\x00\x00\x00\x00";
unsigned char meta_left_0[] = "\x02\x00\x00\x03\x78\x04\x00\x00\x04\x00\x00\x00\xC0\x00\x40\x00\x03\x00\x00\x03\x88\x04\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0F\x0F\x00\x00\x02\x00\x00\x03\x9C\x04\x00\x00\x01\x00\x00\x00\xFF\x3F\x13\x00\x02\x00\x00\x03\xE4\x0C\x00\x00\xE7\x08\x00\x00\xCF\x0B\x00\x00\x01\x00\x00\x03\xE8\x05\x00\x00\x05\x30\x36\x06\x16\x00\x00\x03\xF4\x05\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x20\x38\x18\x3A\x40\xE6\x60\x00\x00\x00\x8F\x02\xC2\x05\x3C\x0A\xFF\xC8\x0F\x00\xFF\xA0\x0F\x00\xFF\xE4\x0F\x00\xFF\xB1\x0F\x00\xFF\xC8\x0F\x00\xFF\xA0\x0F\x00\xFF\xE4\x0F\x00\xFF\xB1\x0F\x00\x0A\x00\x00\x00\x10\x10\x40\x00\x44\xF3\x03\x00\xA0\x00\x40\x00\x66\xE0\x0C\x07\x66\xE0\x0C\x07\xFF\x00\x21\x0A\x00\x02\x52\x07\x24\x0C\x00\x0D\x0B\x00\x00\x03\xBC\x06\x00\x00\x00\x00\x3C\x0B\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x7F\x00\x7F\x00\x0F\x00\x00\xD0\x0F\x00\x00\x10\x00\x00\x00\xC0\x00\xC0\x00\x01\x00\x00\x00\xC0\x00\xC0\x00\x01\x73\x03\x21\x0A\x00\x04\x52\x07\x24\x0C\x00\x0E\x73\x03\x21\x0A\x74\x07\x52\x07\x24\x0C\x00\x0F\x0B\x00\x00\x03\x3C\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x0A\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xE7\x08\x00\x80\xCF\x0B\xDF\x07\x00\x00\x03\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\x68\x0A\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x05\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xE7\x08\x00\x80\xCF\x0B\xEF\x03\x00\x00\x06\x30\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x10\x0E\x00\x00\xE3\x06\xFC\x00\x3F\x05\x00\x00\x02\x00\x00\x03\x30\x0E\x00\x00\x71\x03\x7E\x00\x3F\x05\x00\x00\x02\x00\x00\x03\x18\x0E\x00\x00\x00\x00\xF0\x0F\x16\x00\x00\x00\x02\x00\x00\x03\x38\x0E\x00\x00\x00\x00\xF0\x0F\x17\x00\x00\x00\x0B\x00\x00\x03\xAC\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x7F\x02\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xDB\x08\x00\x80\xCF\x0B\xDF\x01\xCC\x4C\x06\x20\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x0B\x00\x00\x03\xD8\x09\x00\x00\x03\x00\x00\x00\xBF\x0F\x3F\x01\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xDB\x08\x00\x80\xCF\x0B\xEF\x00\xCC\x4C\x06\x10\x00\x00\x00\x00\x00\x00\x00\x00\xCF\x0B\x00\x00\x02\x00\x00\x03\x50\x0E\x00\x00\xA1\x01\x3A\x00\x3F\x01\x00\x00\x02\x00\x00\x03\x70\x0E\x00\x00\xD0\x00\x1D\x00\x3F\x01\x00\x00\x02\x00\x00\x03\x58\x0E\x00\x00\x00\x00\xF0\x0F\x16\x00\x00\x00\x02\x00\x00\x03\x78\x0E\x00\x00\x00\x00\xF0\x0F\x17\x00\x00\x00\x02\x00\x00\x03\x84\x0D\x00\x00\xE3\x06\xFC\x00\xEF\x05\x00\x00\x02\x00\x00\x03\xA4\x0D\x00\x00\x71\x03\x7E\x00\xEF\x05\x00\x00\x03\x00\x00\x03\x60\x0D\x00\x00\x00\x03\xE8\x05\x7D\x6C\x01\x09\x00\x03\xF4\x02\x02\x00\x00\x03\x90\x0D\x00\x00\x79\x01\x00\x00\x4F\x01\x00\x00\x02\x00\x00\x03\xB0\x0D\x00\x00\xBC\x00\x00\x00\x4F\x01\x00\x00\x02\x00\x00\x03\x98\x0D\x00\x00\x00\x00\xF0\x3F\x06\x00\x00\x00\x02\x00\x00\x03\xB8\x0D\x00\x00\x00\x00\xF0\x3F\x07\x00\x00\x00\x01\x00\x00\x03\x6C\x0D\x00\x00\x00\x03\x00\x00\x02\x00\x00\x03\xC4\x0D\x00\x00\x79\x01\x00\x00\x7B\x01\x00\x00\x02\x00\x00\x03\xE4\x0D\x00\x00\xBC\x00\x00\x00\x7B\x01\x00\x00\x03\x00\x00\x03\x70\x0D\x00\x00\x00\x0F\x7A\x01\x7D\x6C\x01\x09\x00\x0F\xBD\x00\x02\x00\x00\x03\xD0\x0D\x00\x00\x5F\x00\x00\x00\x57\x00\x00\x00\x02\x00\x00\x03\xF0\x0D\x00\x00\x2F\x00\x00\x00\x57\x00\x00\x00\x02\x00\x00\x03\xD8\x0D\x00\x00\x00\x00\xF0\x3F\x06\x00\x00\x00\x02\x00\x00\x03\xF8\x0D\x00\x00\x00\x00\xF0\x3F\x07\x00\x00\x00\x01\x00\x00\x03\x7C\x0D\x00\x00\x00\x0F\x00\x00\x07\x00\x00\x03\xB8\x0A\x00\x00\x00\x00\x00\x00\x27\x00\x2F\x00\x37\x00\x37\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x0C\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x98\x0C\x00\x00\x00\xFF\xFF\x00\x07\x00\x00\x03\xC8\x0B\x00\x00\x00\x00\x00\x00\x27\x00\x2F\x00\x37\x00\x37\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x14\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xB0\x0C\x00\x00\x00\xFF\xFF\x00\x02\x00\x00\x03\xD4\x0A\x00\x00\x00\x00\x00\x00\x3D\x04\xE7\x05\x02\x00\x00\x03\xE4\x0B\x00\x00\x00\x00\x00\x00\x3D\x04\x50\x05\x07\x00\x00\x03\xAC\x0B\x00\x00\x00\x00\x00\x00\x10\x00\x17\x00\x7D\x00\x7D\x00\xFF\x3F\xFF\x3F\xFF\x3F\xFF\x3F\x00\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\x10\x0C\x00\x00\x00\x00\x00\x00\x01\x00\x00\x03\xAC\x0C\x00\x00\x00\xFF\xFF\x00\x03\x00\x00\x03\xE4\x0A\x00\x00\x00\x23\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x09\x00\x00\x03\xF0\x0A\x00\x00\x3F\x3E\x3F\x01\x05\x08\x0A\x08\x05\x01\x3F\x3E\x3F\x00\x00\x00\xE9\x05\x00\x00\x17\xFA\xC5\xF8\x94\x6D\xD2\xCB\x3B\x07\x00\x00\xA9\x77\x7B\xC6\x03\x00\x00\x03\x40\x0B\x00\x00\x46\x39\x76\x8D\x46\x39\xC5\xF8\xD5\x71\xC0\xCC\x02\x00\x00\x03\x9C\x0C\x00\x00\x3B\x07\x00\x00\xA9\x77\x7B\xC6\x02\x00\x00\x03\xA4\x0C\x00\x00\x40\x00\x20\x00\xA8\x08\xB0\x0B\x01\x00\x00\x03\x4C\x0B\x00\x00\x0D\x00\x00\x00\x05\x00\x00\x03\x50\x0B\x00\x00\x00\x00\x01\x00\x00\x04\x41\x10\x10\x04\x41\x10\x10\x04\x41\x10\x10\x04\x00\x00\x05\x00\x00\x03\x78\x0B\x00\x00\x00\x00\x01\x00\x00\x04\x41\x10\x10\x04\x41\x10\x10\x04\x41\x10\x10\x04\x00\x00\x02\x00\x00\x03\x8C\x0B\x00\x00\x10\x00\x00\x00\x10\x00\x00\x00\x06\x00\x00\x03\x94\x0B\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x87\x00\x21\x0B\x20\x03\x50\x07\x24\x0C\x00\x40\x7F\x00\x21\x0A\xD0\x14\x52\x07\x24\x0C\x00\x42\x03\x00\x00\x03\xEC\x0B\x00\x00\x00\x00\x00\x00\x07\x00\xEF\x03\xFB\x00\x02\x00\x01\x00\x00\x03\x40\x00\x00\x00\x86\x0D\x00\x00\x01\x00\x00\x03\x48\x00\x00\x00\x0E\x00\x00\x00\x01\x00\x00\x03\x4C\x00\x00\x00\xFF\xFD\x00\x00\x01\x00\x00\x03\x4C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x6C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x0C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x2C\x0E\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x8C\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xAC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xCC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\xEC\x0D\x00\x00\x00\x0E\x00\x00\x01\x00\x00\x03\x44\x00\x00\x00\x3F\x01\x00\x00\x01\x00\x00\x03\xAC\x0A\x00\x00\x48\x0D\x10\x00\x01\x00\x00\x03\x00\x0F\x00\x00\x00\x00\x00\x00";
