import sys
import pandas as pd
import matplotlib.pyplot as plt

def main(csv_path):
    # carrega CSV
    df = pd.read_csv(csv_path)

    # extrai tempo, aceleração, giroscópio e temperatura
    t    = df.iloc[:, 0]
    acc  = df.iloc[:, 1:4]    # colunas 1–3: aceleração X,Y,Z
    gyro = df.iloc[:, 4:7]    # colunas 4–6: giroscópio X,Y,Z
    temp = df.iloc[:, 7]      # coluna 7: temperatura

    # Aceleração
    plt.figure()
    for col in acc.columns:
        plt.plot(t, acc[col], label=col)
    plt.title("Aceleração")
    plt.xlabel("Tempo")
    plt.ylabel("Aceleração")
    plt.legend()
    plt.grid()

    # Giroscópio
    plt.figure()
    for col in gyro.columns:
        plt.plot(t, gyro[col], label=col)
    plt.title("Giroscópio")
    plt.xlabel("Tempo")
    plt.ylabel("Velocidade Angular")
    plt.legend()
    plt.grid()
    # Temperatura
    plt.figure()
    plt.plot(t, temp, 'r-')
    plt.title("Temperatura")
    plt.xlabel("Tempo")
    plt.ylabel("Temperatura (°C)")
    plt.grid()

    plt.show()

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "data1.csv"
    main(path)