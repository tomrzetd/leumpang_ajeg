# Komponen kosong: hanya menyediakan simbol Kconfig yang dirujuk aturan "$CONFIG{...}"
# di manifest registry esp_hosted 3.x. Resolver komponen IDF 5.5 mengevaluasi manifest
# itu walau proyek memakai esp_hosted 2.13.1 lokal; tanpa simbol ini konfigurasi gagal
# ("Missing required kconfig option after retry").
idf_component_register()
