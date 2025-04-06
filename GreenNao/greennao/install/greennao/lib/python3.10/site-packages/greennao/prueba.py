from GreenNaoLib import setV

if __name__ == '__main__':
    # Especifica la velocidad y el archivo de movimientos
    velocity = 1.5  # Velocidad de los movimientos, por ejemplo, 1.5
    file_name = "walk_forwards.csv"  # Asegúrate de que el archivo esté en la ruta correcta
    
    # Llama a la función setV para ejecutar los movimientos
    setV(velocity, file_name)

