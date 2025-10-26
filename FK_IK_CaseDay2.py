import numpy as np              #Untuk perhitungan matriks
import matplotlib.pyplot as plt #Untuk plotting kinematics

# Nama  : Tsaqif Jalaluddin Ahmad
# NIM   : 24/537665/TK/59611 
# Prodi : Teknik Biomedis

# Kode mulai

# ========================================================
# Matriks dan Fungsi untuk Forward Kinematics
# ==2 DoF bidang X-Y==""
def FK_2dof(l1, l2, theta1, theta2):
    theta1 = np.deg2rad(theta1)
    theta2 = np.deg2rad(theta2)

    # R untuk rotasi dan P untuk point di sepanjang sumbu x
    R1 = np.array([ # Array R1 atau rotasi pertama yang akan merotasikan l1
        [np.cos(theta1), -np.sin(theta1), 0],
        [np.sin(theta1), np.cos(theta1), 0],
        [0, 0, 1]
    ])
    P1 = np.array([ # Array P1 berupa panjang lengan atas (yang akan sambung dengan origin)
        [1, 0, l1],
        [0, 1, 0],
        [0, 0, 1]
    ])
    P1 = np.matrix(P1) # Bentuk array R1 dan P1 dirubah ke matriks
    R1 = np.matrix(R1) 

    """Pengubahan ke matriks dilakukan pada baris ini soalnya butuh 
    menjalankan fungsi np.sin dan np.cos, di mana selama saya coding,
    tidak bisa dipanggil jika langsung membentuk dengan np.matrix([])"""
    
    # Pembentukan matriks seperti pada rotasi pertama dan panjang lengan atas (l1)
    R2 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0],
        [np.sin(theta2), np.cos(theta2), 0],
        [0, 0, 1]
    ])
    P2 = np.array([
        [1, 0, l2],
        [0, 1, 0],
        [0, 0, 1]
    ])
    R2 = np.matrix(R2)
    P2 = np.matrix(P2)

    # Hasil rotasi berupa perkalian matriks dengan urutan R1*P1*R2*P2
    # Untuk lebih jelas pada hasilnya terdapat di foto kertas pengerjaan rumus

    hasil = (R1*P1) * (R2*P2)
    return hasil

# ==3 DoF bidang X-Y==
def FK_3dof(l1, l2, l3, theta1, theta2, theta3):
    # Memanggil fungsi sebelumnya untuk menghitung matriks transformasi l1 dengan l2 = T1
    T1 = FK_2dof(l1, l2, theta1, theta2)

    """Konsep perhitungan sama seperti DoF berjumlah 2 karena semua sendi 
    pada kasus ini adalah engsel (2 arah saja) sehingga bagian ke-3 bisa
    dianggap perhitungan 2 DoF antara lengan 2 dengan lengan 3. Hasilnya 
    adalah perhitungan matriks yang mirip, bertambah di bagian akhir saja"""

    theta3 = np.deg2rad(theta3)
    R2 = np.array([
        [np.cos(theta3), -np.sin(theta3), 0],
        [np.sin(theta3), np.cos(theta3), 0],
        [0,0,1]
    ])
    P2 = np.array([
        [1,0,l3],
        [0,1,0],
        [0,0,1]
    ])
    P2 = np.matrix(P2)
    R2 = np.matrix(R2)

    # Perhitungan matrix bertambah sehingga bentuknya menjadi R1*P1*R2*P2*R3*P3 jika dijabarkan
    hasil = T1*(R2*P2)
    return hasil

# ========================================================
# Fungsi untuk Inverse Kinematics
# ==2 DoF bidang X-Y==
def IK_2dof(l1,l2,x,y):
    # Karena bagian sudut dari cos inverse pada kedua rumus (di kertas pengerjaan) didapat sama, yaitu:
    sudut_arccos = (x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2)

    # Misal hasil absolutnya di luar 1, artinya tidak dapat terjangkau fungsi trigonometri, 
    # sehingga pasti titik target di luar jangkauan lengan
    if(abs(sudut_arccos) > 1):
        raise ValueError("Titik berada di luar jangkauan robot, tolong input ulang titik tujuan")

    """Terdapat 2 kasus untuk membentuk pertambahan 2 vektor agar mencapai titik yang sama,
    aturan tersebut disebut paralellogram rule atau aturan jajar genjang (jika pernah dengar)"""

    # Untuk kasus lower-elbow, saat lengan 1 menjadi sisi bawah jajar genjang lalu lengan 2 menunjuk titik target
    theta2_lower = np.arccos(sudut_arccos)
    theta1_lower = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2_lower), l1 + l2*np.cos(theta2_lower))

    # Untuk kasus upper-elbow, saat lengan 2 menjadi sisi kiri jajar genjang lalu lengan 2 menunjuk titik target
    # Perlu diperhatikan di sini sudut dari sin negatif karena lengan 2 harus berputar ke kanan (clockwise) untuk menunjuk titik target
    theta2_upper = -np.arccos(sudut_arccos) 
    theta1_upper = np.arctan2(y,x) - np.arctan2(l2*np.sin(theta2_upper), l1 + l2*np.cos(theta2_upper))

    # Perhitungan lengkap asal usul rumus dapat dilihat di kertas perhitungan
    hasil = np.array([
        [theta1_lower, theta1_upper],
        [theta2_lower, theta2_upper]
    ])
    return hasil

# ========================================================
# Fungsi untuk menampilkan menggambarkan vektor FK atau IK

# Plotting FK 2 dan 3 Dof
def plot_FK(l1,l2,l3,theta1,theta2,theta3):

    # x1 y1 merupakan posisi vektor yang menggambarkan lengan 1 setelah rotasi theta1, dibutuhkan di kedua kasus jumlah DoF
    x1 = l1 * np.cos(np.deg2rad(theta1))
    y1 = l1 * np.sin(np.deg2rad(theta1))

    result = FK_2dof(l1,l2,theta1,theta2)

    x2 = result[0, 2]
    y2 = result[1, 2]

    if (l3 == 0):
    
        # Inisialisasi besar figur untuk plot
        plt.figure(figsize = (10,10))
        plt.title('Ilustrasi 2 Vektor Untuk Lengan')

        # Bentuk vektor lengan atas dari origin (0,0) ke titik x1,y1
        plt.plot([0,x1],[0,y1], color = 'blue', linewidth = 4, label = 'Lengan Atas')
        # Bentuk vektor lengan bawah dengan pangkal di x1,y1 menuju x2,y2 
        plt.plot([x1,x2],[y1,y2], color = 'green', linewidth = 4, label = 'Lengan Bawah')

        plt.plot(0, 0 , color = 'black', marker = 'o', markersize = 10) # Beri tanda lingkaran hitam untuk origin
        plt.plot(x1, y1 , color = 'red', label = 'Sendi 1', marker = 'o', markersize = 10) # Beri tanda lingkaran merah pada sendi di x1,y1
        plt.plot(x2, y2 , color = 'green', marker = 's', markersize = 8) # Tanda persegi sebagai ujung lengan : (x2,y2)

        # Mengambil informasi posisi ujung lengan lalu membulatkannya ke desimal 2 angka di belakang koma dengan round(.., 2)
        info1, info2 = round(x2, 2), (round(y2, 2))
        # Beri informasi titik akhir yang dicapai ujung tangan robot, semibold agar tebal sehingga terlihat lalu posisinya diletakkan di atas
        plt.text(x2, y2, f'({info1},{info2})', position = (info1-12, info2+5), size = 'large', fontweight = 'semibold')

        # sampel lingkaran adalah titik-titik dengan range 0 sampai 2pi lalu dibagi menjadi 150 bagian dengan jarak yang sama
        sampel_lingkaran = np.linspace(0, 2*np.pi, 150)
        # l1+l2 bertindak sebagai jari-jari lingkaran, komponen x terdapat di perkalian dengan cos lalu komponen y dengan sin
        x_lingkaran = (l1+l2)*np.cos(sampel_lingkaran)
        y_lingkaran = (l1+l2)*np.sin(sampel_lingkaran)

        # Titik-titik tersebar yang sudah disampel oleh sampel_lingkaran akan disambungkan semuanya oleh plot di bawah ini
        plt.plot(x_lingkaran, y_lingkaran, linestyle = ':', alpha = 0.55, label = 'Jangkauan Max')
        # Lingkaran akan terbentuk dengan titik-titik (style ':'), transparansi 0.45, lalu diberi keterangan jarak maksimum yang dapat dicapai lengan

        plt.grid(True) # Tampilkan garis grid atau kotak kotak di plot agar lebih jelas perbandingan antar satuannya
        plt.xlim([-(l1+l2+2), l1+l2+2]) # Buat batas sumbu x dan y berupa jari-jari + 2 satuan sehinggao origin pasti di tengah dan gambar jelas
        plt.ylim([-(l1+l2+2), l1+l2+2]) # ini yang sumbu y
        plt.legend(loc = 'upper right', fontsize = 'small')

        plt.plot([-(l1+l2),l1+l2], [0, 0] , color = 'k', linestyle = '--') # Bentuk garis di sumbu x dan y untuk menebalkan sumbu acuan
        plt.plot([0, 0] ,[-(l1+l2),l1+l2], color = 'k', linestyle = '--') # ini yang sumbu y

        plt.show() # tampilkan plot

    else :
        result2 = FK_3dof(l1,l2,l3,theta1,theta2,theta3)

        x3 = result2[0, 2]
        y3 = result2[1, 2]

        plt.figure(figsize = (10,10))
        plt.title('Ilustrasi 3 Vektor Untuk Lengan')

        # Membentuk vektor seperti plotting sebelumnya dengan ketebalan 5 serta label masing-masing
        plt.plot([0,x1],[0,y1], color = 'black', linewidth = 5, label = 'Lengan Atas') # origin -> x1,y1
        plt.plot([x1,x2],[y1,y2], color = 'blue', linewidth = 5, label = 'Lengan Bawah') # x1,y1 -> x2,y2
        plt.plot([x2,x3],[y2,y3], color = 'green', linewidth = 5, label = 'Tangan') # x2,y2 -> x3,y3

        # Memberikan tanda seperti sebelumnya
        plt.plot(0, 0 , color = 'blue', marker = 'o', markersize = 10) # Lingkaran hitam di origin
        plt.plot(x1, y1 , color = 'magenta', label = 'Sendi 1', marker = 'o', markersize = 10) # lingkaran magenta : sendi pertama
        plt.plot(x2, y2 , color = 'red', label = 'Sendi 2',marker = 'o', markersize = 10) # lingkaran merah : sendi kedua
        plt.plot(x3, y3 , color = 'blue', marker = 's', markersize = 7) # persegi tanda ujung tangan

        info1, info2 = round(x3, 2), (round(y3, 2)) # Menampilkan informasi letak akhir tangan seperti sebelumnya
        plt.text(x2, y2, f'({info1}, {info2})', position = (info1-12, info2+5), size = 'large', fontweight = 'semibold')

        # Sisa kode persis seperti pada plotting kasus 2 DoF
        sampel_lingkaran = np.linspace(0, 2*np.pi, 150)
        x_lingkaran = (l1+l2+l3)*np.cos(sampel_lingkaran)
        y_lingkaran = (l1+l2+l3)*np.sin(sampel_lingkaran)
        plt.plot(x_lingkaran, y_lingkaran, linestyle = ':', alpha = 0.55, label = 'Jangkauan Max')

        plt.grid(True)
        plt.xlim([-(l1+l2+l3+2), l1+l2+l3+2])
        plt.ylim([-(l1+l2+l3+2), l1+l2+l3+2])
        plt.legend(loc = 'upper right', fontsize = 'small')

        plt.plot([-(l1+l2+l3),l1+l2+l3], [0, 0] , color = 'k', linestyle = '--')
        plt.plot([0, 0] ,[-(l1+l2+l3),l1+l2+l3], color = 'k', linestyle = '--')

        plt.show()

# Plotting IK 2 Dof
def plot_IK(l1, l2, x, y):
    result = IK_2dof(l1, l2, x, y)

    # Derajat (rad) dan Vektor Lower Elbow
    low_theta1 = result[0, 0]
    low_theta2 = result[1, 0]

    x1 = l1 * np.cos(low_theta1)
    y1 = l1 * np.sin(low_theta1)

    x2 = l1 * np.cos(low_theta1) + l2 * np.cos(low_theta1 + low_theta2)
    y2 = l1 * np.sin(low_theta1) + l2 * np.sin(low_theta1 + low_theta2)

    # Derajat (rad) dan Vektor Upper Elbow
    up_theta1 = result[0, 1]
    up_theta2 = result[1, 1]

    x3 = l1 * np.cos(up_theta1)
    y3 = l1 * np.sin(up_theta1)

    x4 = l1 * np.cos(up_theta1) + l2 * np.cos(up_theta1 + up_theta2)
    y4 = l1 * np.sin(up_theta1) + l2 * np.sin(up_theta1 + up_theta2)
    """Pada bagian ini tidak bisa memanggil fungsi plot untuk masing-masing bagian upper elbow dan
    lower elbow karena nantinya akan terpisah menjadi 2 window berbeda dan sulit dibandingkan kedua 
    solusi tersebut"""

    # Gambar plot
    plt.figure(figsize = (10,10))
    plt.title(f'Ilustrasi 2 Vektor Posisi : {x}, {y}')

    # Untuk menggambar vektor lower elbow
    plt.plot([0,x1],[0,y1], color = 'black', linewidth = 4, label = 'Bagian_1 Lower') # Vektor origin -> x1,y1
    plt.plot([x1,x2],[y1,y2], color = 'blue', linewidth = 4, label = 'Bagian_2 Lower') # Vektor x1,y1 -> x2,y2

    # Untuk menggambar vektor upper elbow
    plt.plot([0,x3],[0,y3], color = 'magenta', linewidth = 4, label = 'Bagian_1 Upper') # Vektor origin -> x3,y3
    plt.plot([x3,x4],[y3,y4], color = 'green', linewidth = 4, label = 'Bagian_2 Upper') # Vektor x3,y3 -> x4,y4

    plt.plot(0, 0 , color = 'black', marker = 'o', markersize = 10) # Penanda origin
    plt.plot(x1, y1 , color = 'red', marker = 'o', markersize = 10) # Penanda sendi lower elbow
    plt.plot(x3, y3 , color = 'red', marker = 'o', markersize = 10) # Penanda sendi upper elbow

    plt.plot(x2, y2 , color = 'black', marker = 'o', markersize = 6) # Penanda ujung tangan lower elbow
    plt.plot(x4, y4 , color = 'black', marker = 'o', markersize = 6) # Penanda ujung tangan upper elbow

    info1, info2 = round(x, 2), (round(y, 2)) # Informasi titik akhir
    plt.text(x2, y2, f'({info1},{info2})', position = (info1-12, info2+5), size = 'large', fontweight = 'semibold')

    sampel_lingkaran = np.linspace(0, 2*np.pi, 150)
    x_lingkaran = (l1+l2)*np.cos(sampel_lingkaran)
    y_lingkaran = (l1+l2)*np.sin(sampel_lingkaran)
    plt.plot(x_lingkaran, y_lingkaran, linestyle = ':', alpha = 0.55)

    plt.grid(True)
    plt.xlim([-(l1+l2+2), l1+l2+2])
    plt.ylim([-(l1+l2+2), l1+l2+2])
    plt.legend(loc = 'upper right', fontsize = 'small')

    plt.plot([-(l1+l2),l1+l2], [0, 0] , color = 'k', linestyle = '--')
    plt.plot([0, 0] ,[-(l1+l2),l1+l2], color = 'k', linestyle = '--')

    plt.show()

# ========================================================
# Untuk interaksi dengan user
if __name__ == "__main__" :
    print("=== Hitung Forward Kinematic atau Inverse Kinematic ===")
    print("Pilih jenis operasi:")
    print("1. Forward Kinematics (FK)")
    print("2. Inverse Kinematics (IK)")
    
    try:
        pilihan = input("Masukkan pilihan (1 atau 2): ").strip()
        
        if (pilihan == '1'): 
            print("\n--- Menu Forward Kinematics ---")
            jumlah_dof = input("Jumlah DOF (2 atau 3): ").strip()

            l1 = float(input("Panjang lengan 1 (l1), mulai dari origin hingga sendi pertama: "))
            l2 = float(input("Panjang lengan 2 (l2): "))
            l3 = 0.0

            if (jumlah_dof == '3') :
                l3 = float(input("Panjang lengan 3 (l3): "))
            elif (jumlah_dof != '2') :
                raise ValueError("Opsi DoF hanya bisa 2 atau 3 untuk FK.")
            
            theta1 = float(input("Sudut theta1 (satuan derajat): "))
            theta2 = float(input("Sudut theta2 (satuan derajat): "))
            theta3 = 0.0

            if jumlah_dof == '3':
                theta3 = float(input("Sudut theta3 (derajat): "))
            
            # Menampilkan dari matrix transformasi di mana terdapat solusi di hasilnya lalu pop gambar plotnya
            if jumlah_dof == '2':
                Transformation_matrix = FK_2dof(l1, l2, theta1, theta2)
                print(f"\nMatriks Transformasi FK 2-DOF:\n{Transformation_matrix}")
                plot_FK(l1, l2, 0, theta1, theta2, 0)
            else:
                Transformation_matrix = FK_3dof(l1, l2, l3, theta1, theta2, theta3)
                print(f"\nMatriks Transformasi FK 3-DOF:\n{Transformation_matrix}")
                plot_FK(l1, l2, l3, theta1, theta2, theta3)
                
        elif (pilihan == '2'):
            print("\n--- Inverse Kinematics (2-DOF) ---")
            l1 = float(input("Panjang lengan 1 (l1), mulai dari origin hingga sendi pertama: "))
            l2 = float(input("Panjang lengan 2 (l2): "))
            x  = float(input("Koordinat target x: "))
            y  = float(input("Koordinat target y: "))
            
            try:
                sudut = IK_2dof(l1, l2, x, y)

                low_theta1 = sudut[0, 0]
                low_theta2 = sudut[1, 0]

                up_theta1 = sudut[0, 1]
                up_theta2 = sudut[1, 1]
                
                print(f"\nSolusi kasus Lower Elbow:  theta1 = {low_theta1:.2f}, theta2 = {low_theta2:.2f}")
                print(f"Solusi kasus Upper Elbow:  theta1 = {up_theta1:.2f}, theta2 = {up_theta2:.2f}")
                print("dengan satuan berupa derajat")
                
                plot_IK(l1, l2, x, y)
                
            except ValueError as e:
                print(f"Error: {e}")
        else:
            print("Pilihan tidak valid. Jalankan ulang program.")
            
    except Exception as e:
        print(f"Input tidak valid: {e}")
        print("Pastikan memasukkan angka yang benar.")

# Kode selesai