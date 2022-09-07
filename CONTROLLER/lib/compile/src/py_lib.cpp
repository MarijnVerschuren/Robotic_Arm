#include "py_lib.hpp"



#define ceil(x) (((int)(x)) == ((float)(x))) ? ((int)(x)) : (((int)(x)) + 1)
#define floor(x) ((int)(x))
#define max(x, y) (x > y) ? x : y
#define min(x, y) (x < y) ? x : y


constexpr uint64 CRC_64_ECMA_TABLE[256] {
	0x0000000000000000, 0x42f0e1eba9ea3693, 0x85e1c3d753d46d26, 0xc711223cfa3e5bb5, 0x493366450e42ecdf, 0x0bc387aea7a8da4c, 0xccd2a5925d9681f9, 0x8e224479f47cb76a,
	0x9266cc8a1c85d9be, 0xd0962d61b56fef2d, 0x17870f5d4f51b498, 0x5577eeb6e6bb820b, 0xdb55aacf12c73561, 0x99a54b24bb2d03f2, 0x5eb4691841135847, 0x1c4488f3e8f96ed4,
	0x663d78ff90e185ef, 0x24cd9914390bb37c, 0xe3dcbb28c335e8c9, 0xa12c5ac36adfde5a, 0x2f0e1eba9ea36930, 0x6dfeff5137495fa3, 0xaaefdd6dcd770416, 0xe81f3c86649d3285,
	0xf45bb4758c645c51, 0xb6ab559e258e6ac2, 0x71ba77a2dfb03177, 0x334a9649765a07e4, 0xbd68d2308226b08e, 0xff9833db2bcc861d, 0x388911e7d1f2dda8, 0x7a79f00c7818eb3b,
	0xcc7af1ff21c30bde, 0x8e8a101488293d4d, 0x499b3228721766f8, 0x0b6bd3c3dbfd506b, 0x854997ba2f81e701, 0xc7b97651866bd192, 0x00a8546d7c558a27, 0x4258b586d5bfbcb4,
	0x5e1c3d753d46d260, 0x1cecdc9e94ace4f3, 0xdbfdfea26e92bf46, 0x990d1f49c77889d5, 0x172f5b3033043ebf, 0x55dfbadb9aee082c, 0x92ce98e760d05399, 0xd03e790cc93a650a,
	0xaa478900b1228e31, 0xe8b768eb18c8b8a2, 0x2fa64ad7e2f6e317, 0x6d56ab3c4b1cd584, 0xe374ef45bf6062ee, 0xa1840eae168a547d, 0x66952c92ecb40fc8, 0x2465cd79455e395b,
	0x3821458aada7578f, 0x7ad1a461044d611c, 0xbdc0865dfe733aa9, 0xff3067b657990c3a, 0x711223cfa3e5bb50, 0x33e2c2240a0f8dc3, 0xf4f3e018f031d676, 0xb60301f359dbe0e5,
	0xda050215ea6c212f, 0x98f5e3fe438617bc, 0x5fe4c1c2b9b84c09, 0x1d14202910527a9a, 0x93366450e42ecdf0, 0xd1c685bb4dc4fb63, 0x16d7a787b7faa0d6, 0x5427466c1e109645,
	0x4863ce9ff6e9f891, 0x0a932f745f03ce02, 0xcd820d48a53d95b7, 0x8f72eca30cd7a324, 0x0150a8daf8ab144e, 0x43a04931514122dd, 0x84b16b0dab7f7968, 0xc6418ae602954ffb,
	0xbc387aea7a8da4c0, 0xfec89b01d3679253, 0x39d9b93d2959c9e6, 0x7b2958d680b3ff75, 0xf50b1caf74cf481f, 0xb7fbfd44dd257e8c, 0x70eadf78271b2539, 0x321a3e938ef113aa,
	0x2e5eb66066087d7e, 0x6cae578bcfe24bed, 0xabbf75b735dc1058, 0xe94f945c9c3626cb, 0x676dd025684a91a1, 0x259d31cec1a0a732, 0xe28c13f23b9efc87, 0xa07cf2199274ca14,
	0x167ff3eacbaf2af1, 0x548f120162451c62, 0x939e303d987b47d7, 0xd16ed1d631917144, 0x5f4c95afc5edc62e, 0x1dbc74446c07f0bd, 0xdaad56789639ab08, 0x985db7933fd39d9b,
	0x84193f60d72af34f, 0xc6e9de8b7ec0c5dc, 0x01f8fcb784fe9e69, 0x43081d5c2d14a8fa, 0xcd2a5925d9681f90, 0x8fdab8ce70822903, 0x48cb9af28abc72b6, 0x0a3b7b1923564425,
	0x70428b155b4eaf1e, 0x32b26afef2a4998d, 0xf5a348c2089ac238, 0xb753a929a170f4ab, 0x3971ed50550c43c1, 0x7b810cbbfce67552, 0xbc902e8706d82ee7, 0xfe60cf6caf321874,
	0xe224479f47cb76a0, 0xa0d4a674ee214033, 0x67c58448141f1b86, 0x253565a3bdf52d15, 0xab1721da49899a7f, 0xe9e7c031e063acec, 0x2ef6e20d1a5df759, 0x6c0603e6b3b7c1ca,
	0xf6fae5c07d3274cd, 0xb40a042bd4d8425e, 0x731b26172ee619eb, 0x31ebc7fc870c2f78, 0xbfc9838573709812, 0xfd39626eda9aae81, 0x3a28405220a4f534, 0x78d8a1b9894ec3a7,
	0x649c294a61b7ad73, 0x266cc8a1c85d9be0, 0xe17dea9d3263c055, 0xa38d0b769b89f6c6, 0x2daf4f0f6ff541ac, 0x6f5faee4c61f773f, 0xa84e8cd83c212c8a, 0xeabe6d3395cb1a19,
	0x90c79d3fedd3f122, 0xd2377cd44439c7b1, 0x15265ee8be079c04, 0x57d6bf0317edaa97, 0xd9f4fb7ae3911dfd, 0x9b041a914a7b2b6e, 0x5c1538adb04570db, 0x1ee5d94619af4648,
	0x02a151b5f156289c, 0x4051b05e58bc1e0f, 0x87409262a28245ba, 0xc5b073890b687329, 0x4b9237f0ff14c443, 0x0962d61b56fef2d0, 0xce73f427acc0a965, 0x8c8315cc052a9ff6,
	0x3a80143f5cf17f13, 0x7870f5d4f51b4980, 0xbf61d7e80f251235, 0xfd913603a6cf24a6, 0x73b3727a52b393cc, 0x31439391fb59a55f, 0xf652b1ad0167feea, 0xb4a25046a88dc879,
	0xa8e6d8b54074a6ad, 0xea16395ee99e903e, 0x2d071b6213a0cb8b, 0x6ff7fa89ba4afd18, 0xe1d5bef04e364a72, 0xa3255f1be7dc7ce1, 0x64347d271de22754, 0x26c49cccb40811c7,
	0x5cbd6cc0cc10fafc, 0x1e4d8d2b65facc6f, 0xd95caf179fc497da, 0x9bac4efc362ea149, 0x158e0a85c2521623, 0x577eeb6e6bb820b0, 0x906fc95291867b05, 0xd29f28b9386c4d96,
	0xcedba04ad0952342, 0x8c2b41a1797f15d1, 0x4b3a639d83414e64, 0x09ca82762aab78f7, 0x87e8c60fded7cf9d, 0xc51827e4773df90e, 0x020905d88d03a2bb, 0x40f9e43324e99428,
	0x2cffe7d5975e55e2, 0x6e0f063e3eb46371, 0xa91e2402c48a38c4, 0xebeec5e96d600e57, 0x65cc8190991cb93d, 0x273c607b30f68fae, 0xe02d4247cac8d41b, 0xa2dda3ac6322e288,
	0xbe992b5f8bdb8c5c, 0xfc69cab42231bacf, 0x3b78e888d80fe17a, 0x7988096371e5d7e9, 0xf7aa4d1a85996083, 0xb55aacf12c735610, 0x724b8ecdd64d0da5, 0x30bb6f267fa73b36,
	0x4ac29f2a07bfd00d, 0x08327ec1ae55e69e, 0xcf235cfd546bbd2b, 0x8dd3bd16fd818bb8, 0x03f1f96f09fd3cd2, 0x41011884a0170a41, 0x86103ab85a2951f4, 0xc4e0db53f3c36767,
	0xd8a453a01b3a09b3, 0x9a54b24bb2d03f20, 0x5d45907748ee6495, 0x1fb5719ce1045206, 0x919735e51578e56c, 0xd367d40ebc92d3ff, 0x1476f63246ac884a, 0x568617d9ef46bed9,
	0xe085162ab69d5e3c, 0xa275f7c11f7768af, 0x6564d5fde549331a, 0x279434164ca30589, 0xa9b6706fb8dfb2e3, 0xeb46918411358470, 0x2c57b3b8eb0bdfc5, 0x6ea7525342e1e956,
	0x72e3daa0aa188782, 0x30133b4b03f2b111, 0xf7021977f9cceaa4, 0xb5f2f89c5026dc37, 0x3bd0bce5a45a6b5d, 0x79205d0e0db05dce, 0xbe317f32f78e067b, 0xfcc19ed95e6430e8,
	0x86b86ed5267cdbd3, 0xc4488f3e8f96ed40, 0x0359ad0275a8b6f5, 0x41a94ce9dc428066, 0xcf8b0890283e370c, 0x8d7be97b81d4019f, 0x4a6acb477bea5a2a, 0x089a2aacd2006cb9,
	0x14dea25f3af9026d, 0x562e43b4931334fe, 0x913f6188692d6f4b, 0xd3cf8063c0c759d8, 0x5dedc41a34bbeeb2, 0x1f1d25f19d51d821, 0xd80c07cd676f8394, 0x9afce626ce85b507
};

constexpr uint32 CRC_32_K_TABLE[256] = {
	0x00000000, 0x741b8cd7, 0xe83719ae, 0x9c2c9579, 0xa475bf8b, 0xd06e335c, 0x4c42a625, 0x38592af2,
	0x3cf0f3c1, 0x48eb7f16, 0xd4c7ea6f, 0xa0dc66b8, 0x98854c4a, 0xec9ec09d, 0x70b255e4, 0x04a9d933,
	0x79e1e782, 0x0dfa6b55, 0x91d6fe2c, 0xe5cd72fb, 0xdd945809, 0xa98fd4de, 0x35a341a7, 0x41b8cd70,
	0x45111443, 0x310a9894, 0xad260ded, 0xd93d813a, 0xe164abc8, 0x957f271f, 0x0953b266, 0x7d483eb1,
	0xf3c3cf04, 0x87d843d3, 0x1bf4d6aa, 0x6fef5a7d, 0x57b6708f, 0x23adfc58, 0xbf816921, 0xcb9ae5f6,
	0xcf333cc5, 0xbb28b012, 0x2704256b, 0x531fa9bc, 0x6b46834e, 0x1f5d0f99, 0x83719ae0, 0xf76a1637,
	0x8a222886, 0xfe39a451, 0x62153128, 0x160ebdff, 0x2e57970d, 0x5a4c1bda, 0xc6608ea3, 0xb27b0274,
	0xb6d2db47, 0xc2c95790, 0x5ee5c2e9, 0x2afe4e3e, 0x12a764cc, 0x66bce81b, 0xfa907d62, 0x8e8bf1b5,
	0x939c12df, 0xe7879e08, 0x7bab0b71, 0x0fb087a6, 0x37e9ad54, 0x43f22183, 0xdfdeb4fa, 0xabc5382d,
	0xaf6ce11e, 0xdb776dc9, 0x475bf8b0, 0x33407467, 0x0b195e95, 0x7f02d242, 0xe32e473b, 0x9735cbec,
	0xea7df55d, 0x9e66798a, 0x024aecf3, 0x76516024, 0x4e084ad6, 0x3a13c601, 0xa63f5378, 0xd224dfaf,
	0xd68d069c, 0xa2968a4b, 0x3eba1f32, 0x4aa193e5, 0x72f8b917, 0x06e335c0, 0x9acfa0b9, 0xeed42c6e,
	0x605fdddb, 0x1444510c, 0x8868c475, 0xfc7348a2, 0xc42a6250, 0xb031ee87, 0x2c1d7bfe, 0x5806f729,
	0x5caf2e1a, 0x28b4a2cd, 0xb49837b4, 0xc083bb63, 0xf8da9191, 0x8cc11d46, 0x10ed883f, 0x64f604e8,
	0x19be3a59, 0x6da5b68e, 0xf18923f7, 0x8592af20, 0xbdcb85d2, 0xc9d00905, 0x55fc9c7c, 0x21e710ab,
	0x254ec998, 0x5155454f, 0xcd79d036, 0xb9625ce1, 0x813b7613, 0xf520fac4, 0x690c6fbd, 0x1d17e36a,
	0x5323a969, 0x273825be, 0xbb14b0c7, 0xcf0f3c10, 0xf75616e2, 0x834d9a35, 0x1f610f4c, 0x6b7a839b,
	0x6fd35aa8, 0x1bc8d67f, 0x87e44306, 0xf3ffcfd1, 0xcba6e523, 0xbfbd69f4, 0x2391fc8d, 0x578a705a,
	0x2ac24eeb, 0x5ed9c23c, 0xc2f55745, 0xb6eedb92, 0x8eb7f160, 0xfaac7db7, 0x6680e8ce, 0x129b6419,
	0x1632bd2a, 0x622931fd, 0xfe05a484, 0x8a1e2853, 0xb24702a1, 0xc65c8e76, 0x5a701b0f, 0x2e6b97d8,
	0xa0e0666d, 0xd4fbeaba, 0x48d77fc3, 0x3cccf314, 0x0495d9e6, 0x708e5531, 0xeca2c048, 0x98b94c9f,
	0x9c1095ac, 0xe80b197b, 0x74278c02, 0x003c00d5, 0x38652a27, 0x4c7ea6f0, 0xd0523389, 0xa449bf5e,
	0xd90181ef, 0xad1a0d38, 0x31369841, 0x452d1496, 0x7d743e64, 0x096fb2b3, 0x954327ca, 0xe158ab1d,
	0xe5f1722e, 0x91eafef9, 0x0dc66b80, 0x79dde757, 0x4184cda5, 0x359f4172, 0xa9b3d40b, 0xdda858dc,
	0xc0bfbbb6, 0xb4a43761, 0x2888a218, 0x5c932ecf, 0x64ca043d, 0x10d188ea, 0x8cfd1d93, 0xf8e69144,
	0xfc4f4877, 0x8854c4a0, 0x147851d9, 0x6063dd0e, 0x583af7fc, 0x2c217b2b, 0xb00dee52, 0xc4166285,
	0xb95e5c34, 0xcd45d0e3, 0x5169459a, 0x2572c94d, 0x1d2be3bf, 0x69306f68, 0xf51cfa11, 0x810776c6,
	0x85aeaff5, 0xf1b52322, 0x6d99b65b, 0x19823a8c, 0x21db107e, 0x55c09ca9, 0xc9ec09d0, 0xbdf78507,
	0x337c74b2, 0x4767f865, 0xdb4b6d1c, 0xaf50e1cb, 0x9709cb39, 0xe31247ee, 0x7f3ed297, 0x0b255e40,
	0x0f8c8773, 0x7b970ba4, 0xe7bb9edd, 0x93a0120a, 0xabf938f8, 0xdfe2b42f, 0x43ce2156, 0x37d5ad81,
	0x4a9d9330, 0x3e861fe7, 0xa2aa8a9e, 0xd6b10649, 0xeee82cbb, 0x9af3a06c, 0x06df3515, 0x72c4b9c2,
	0x766d60f1, 0x0276ec26, 0x9e5a795f, 0xea41f588, 0xd218df7a, 0xa60353ad, 0x3a2fc6d4, 0x4e344a03
};

constexpr uint16 CRC_16_TABLE[256] = {
	0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011,
	0x8033, 0x0036, 0x003c, 0x8039, 0x0028, 0x802d, 0x8027, 0x0022,
	0x8063, 0x0066, 0x006c, 0x8069, 0x0078, 0x807d, 0x8077, 0x0072,
	0x0050, 0x8055, 0x805f, 0x005a, 0x804b, 0x004e, 0x0044, 0x8041,
	0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2,
	0x00f0, 0x80f5, 0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1,
	0x00a0, 0x80a5, 0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1,
	0x8093, 0x0096, 0x009c, 0x8099, 0x0088, 0x808d, 0x8087, 0x0082,
	0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d, 0x8197, 0x0192,
	0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1,
	0x01e0, 0x81e5, 0x81ef, 0x01ea, 0x81fb, 0x01fe, 0x01f4, 0x81f1,
	0x81d3, 0x01d6, 0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2,
	0x0140, 0x8145, 0x814f, 0x014a, 0x815b, 0x015e, 0x0154, 0x8151,
	0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d, 0x8167, 0x0162,
	0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132,
	0x0110, 0x8115, 0x811f, 0x011a, 0x810b, 0x010e, 0x0104, 0x8101,
	0x8303, 0x0306, 0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312,
	0x0330, 0x8335, 0x833f, 0x033a, 0x832b, 0x032e, 0x0324, 0x8321,
	0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e, 0x0374, 0x8371,
	0x8353, 0x0356, 0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342,
	0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1,
	0x83f3, 0x03f6, 0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2,
	0x83a3, 0x03a6, 0x03ac, 0x83a9, 0x03b8, 0x83bd, 0x83b7, 0x03b2,
	0x0390, 0x8395, 0x839f, 0x039a, 0x838b, 0x038e, 0x0384, 0x8381,
	0x0280, 0x8285, 0x828f, 0x028a, 0x829b, 0x029e, 0x0294, 0x8291,
	0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2,
	0x82e3, 0x02e6, 0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2,
	0x02d0, 0x82d5, 0x82df, 0x02da, 0x82cb, 0x02ce, 0x02c4, 0x82c1,
	0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d, 0x8257, 0x0252,
	0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e, 0x0264, 0x8261,
	0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231,
	0x8213, 0x0216, 0x021c, 0x8219, 0x0208, 0x820d, 0x8207, 0x0202
};

constexpr uint8 CRC_8_TABLE[256] = {
	0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54,
	0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
	0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06,
	0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
	0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0,
	0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
	0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2,
	0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
	0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9,
	0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
	0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b,
	0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
	0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d,
	0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
	0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f,
	0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
	0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb,
	0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
	0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9,
	0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
	0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f,
	0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
	0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d,
	0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
	0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26,
	0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
	0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74,
	0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
	0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82,
	0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
	0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0,
	0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

constexpr uint8 CRC_4_ITU_TABLE[16] = {
	0x0, 0x3, 0x6, 0x5, 0xd, 0xe, 0xb, 0x8,
	0xa, 0x9, 0xc, 0xf, 0x7, 0x4, 0x1, 0x2
};



/* shared */
const uint64 crc_64(const int8* buffer, uint64 bytes) {
	uint64 crc = 0x0000000000000000ull;
	for (uint64 i = 0; i < bytes; i++) { crc = (crc << 8) ^ CRC_64_ECMA_TABLE[((crc >> 8) ^ ((uint8*)buffer)[i]) & 0xff]; }
	return crc;
}
const uint32 crc_32(const int8* buffer, uint64 bytes) {
	uint32 crc = 0x00000000ul;
	for (uint64 i = 0; i < bytes; i++) { crc = (crc << 8) ^ CRC_32_K_TABLE[((crc >> 8) ^ ((uint8*)buffer)[i]) & 0xff]; }
	return crc;
}
const uint16 crc_16(const int8* buffer, uint64 bytes) {
	uint16 crc = 0x0000u;
	for (uint64 i = 0; i < bytes; i++) { crc = (crc << 8) ^ CRC_16_TABLE[(crc >> 8) ^ ((uint8*)buffer)[i]]; }
	return crc;
}
const uint8 crc_8(const int8* buffer, uint64 bytes) {
	uint8 crc = 0x00u;
	for (uint64 i = 0; i < bytes; i++) { crc = (crc << 8) ^ CRC_8_TABLE[(crc >> 8) ^ ((uint8*)buffer)[i]]; }
	return crc;
}
const uint8 crc_4(const int8* buffer, uint64 bytes) {
	uint64 loop = bytes * 2; uint8 crc = 0x0u;
	for (uint64 i = 0; i < loop; i++) { crc = (crc >> 2) ^ CRC_4_ITU_TABLE[crc ^ ((((uint8*)buffer)[(uint)(i/2)] >> (4 * (i % 2))) & 0xf)]; }
	return crc;
}


typedef const uint64(*crc_functype) (const int8*, uint64);
bool error_correct(int8* buffer, uint64 bytes, uint64& crc, uint8 type, uint64 bit_cutoff) {
	crc_functype f; uint8 crc_bits = max(type * 8, 4); uint64 bit_mask;
	/* this switch statement selects the crc funtion according to the given type
	 * the selected function is cast to a function type that returns the largest integer so that the output of any crc function is captured
	 * then a bit_mask is set so that non data bits can be masked because casting a function to functype with a different return type has undefined behaviour */
	switch (type) {
	case CRC_TYPE::_::CRC_4:	f = (crc_functype)crc_4;		bit_mask = 0xf;					break;
	case CRC_TYPE::_::CRC_8:	f = (crc_functype)crc_8;		bit_mask = 0xff;				break;
	case CRC_TYPE::_::CRC_16:	f = (crc_functype)crc_16;		bit_mask = 0xffff;				break;
	case CRC_TYPE::_::CRC_32:	f = (crc_functype)crc_32;		bit_mask = 0xffffffff;			break;
	case CRC_TYPE::_::CRC_64:	f = (crc_functype)crc_64;		bit_mask = 0xffffffffffffffff;	break;
	}

	uint64 current_crc = f(buffer, bytes) & bit_mask;
	if (current_crc ^ crc) {
		if (!bit_cutoff) { bit_cutoff = 8 * bytes; }  // when bit_cutoff is not given it will be set to all bits in the
		uint64 i = 0; uint8 flip = 0x01; uint8* ptr = (uint8*)buffer - 1;
		for (; i < bit_cutoff; i++, flip = 0x01 << (i % 8)) {
			if (flip == 0x01) { ptr++; } (*ptr) ^= flip; if (!((f(buffer, bytes) ^ crc) & bit_mask)) { return true; } (*ptr) ^= flip;
		} uint64 crc_buf = crc ^ 0x0000000000000001ull;
		for (i = 0; i < crc_bits; i++, crc_buf = crc ^ (0x0000000000000001ull << i)) {
			if (!((current_crc ^ crc_buf) & bit_mask)) { crc = crc_buf; return true; }
		} return false;
	} return true;
}

void header::set(uint8 op_code, uint8 msg_id, uint8 motor_id, uint8 poly_data) {
	this->data.op_code = op_code; this->data.msg_id = msg_id;
	this->data.motor_id = motor_id; this->data.poly_data = poly_data;
	this->data.crc = crc_16((int8*)this, 2);
}
bool header::load(uint32 value) {
	this->_raw = value;
	uint64 crc = this->data.crc;
	if (error_correct((int8*)this, 2, crc, CRC_TYPE::_::CRC_16)) {
		this->data.crc = crc & 0xffff; return true;
	} return false;
}

void motor_instruction::set(int64 steps, uint32 pulse_delay) {
	this->steps = steps;
	this->pulse_delay = pulse_delay;
	this->crc = crc_32((int8*)this, 12);
}

bool MCU_state::load(const int8* buffer) {
	this->pos = ((int64*)buffer)[0];
	this->job = ((int64*)buffer)[1];
	uint64 crc = ((uint16*)buffer)[8];
	if (error_correct((int8*)this, 16, crc, CRC_TYPE::_::CRC_16)) {
		this->crc = crc & 0xffff; return true;
	} return false;
}
// https://www.researchgate.net/figure/Illustration-of-the-single-error-search-applied-to-CRC-4-ITU-where-gx-x-4-x-1_fig4_341812360
// TODO ^