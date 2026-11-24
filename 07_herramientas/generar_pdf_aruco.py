#!/usr/bin/env python3
"""Genera la hoja A4 lista para imprimir al 100% (escala 1:1) con:
  1) 4 marcadores ArUco (DICT_4X4_50, IDs 0, 1, 2, 3) de 25 x 25 mm para las 4 esquinas
     de la bandeja Ball & Plate (utilizados para homografia y medicion de inclinacion).
  2) 4 marcadores ArUco (DICT_4X4_50, IDs 10, 11, 12, 13) de 40 x 40 mm para seguimiento
     externo de altura z e inclinacion del dron octocoptero (comparacion con Isaac Lab).
  3) Cruces de referencia y regla milimetrada de verificacion de escala (100 mm).
"""
import os
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SALIDA_DIR = os.path.join(RAIZ, "imprimibles")
os.makedirs(SALIDA_DIR, exist_ok=True)
RUTA_PDF = os.path.join(SALIDA_DIR, "Marcadores_ArUco_y_Calibracion_Bandeja_A4.pdf")
RUTA_PNG = os.path.join(SALIDA_DIR, "Marcadores_ArUco_y_Calibracion_Bandeja_A4.png")


def generar_aruco(id_marcador, px=400):
    dic = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50) if hasattr(cv2.aruco, "Dictionary_get") \
        else cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "drawMarker"):
        return cv2.aruco.drawMarker(dic, id_marcador, px)
    return cv2.aruco.generateImageMarker(dic, id_marcador, px)


def dibujar_marcador(ax, id_m, x_mm, y_mm, lado_mm, etiqueta):
    img = generar_aruco(id_m, 400)
    # Margen blanco de 3 mm alrededor para que el detector ArUco funcione sobre fondo oscuro/rojo
    m = 3.0
    ax.add_patch(Rectangle((x_mm - m, y_mm - m), lado_mm + 2 * m, lado_mm + 2 * m,
                           facecolor="white", edgecolor="#555555", linestyle="--", linewidth=0.8))
    ax.imshow(img, cmap="gray", vmin=0, vmax=255,
              extent=[x_mm, x_mm + lado_mm, y_mm, y_mm + lado_mm], origin="upper", zorder=3)
    ax.text(x_mm + lado_mm / 2, y_mm - m - 2.5,
            f"ID {id_m} ({lado_mm:.0f} mm)\n{etiqueta}",
            ha="center", va="top", fontsize=6.8, fontweight="bold")


def main():
    # Hoja A4 en milimetros: 210 x 297 mm
    fig = plt.figure(figsize=(210 / 25.4, 297 / 25.4), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 210)
    ax.set_ylim(0, 297)
    ax.set_aspect("equal")
    ax.axis("off")

    # Encabezado
    ax.text(105, 287, "DSM - MARCADORES ARUCO (DICT_4X4_50) Y CALIBRACION DE CAMARA",
            ha="center", va="top", fontsize=11, fontweight="bold")
    ax.text(105, 281,
            "Imprimir en A4 a ESCALA 100% (Tamano real / Sin ajustar a pagina). Recortar por la linea punteada dejando el borde blanco.",
            ha="center", va="top", fontsize=8)

    # --- SECCION 1: 4 Marcadores de 25 mm para las esquinas de la bandeja Ball & Plate ---
    ax.text(15, 270, "1) ESQUINAS DE LA BANDEJA BALL & PLATE (25 x 25 mm, pegar fuera de la zona negra donde rueda la bola):",
            ha="left", va="top", fontsize=8.5, fontweight="bold")

    esquinas = [
        (0, 20, 232, "Sup-Izq (-X, +Y)"),
        (1, 68, 232, "Sup-Der (+X, +Y)"),
        (2, 116, 232, "Inf-Der (+X, -Y)"),
        (3, 164, 232, "Inf-Izq (-X, -Y)"),
    ]
    for id_m, x, y, etq in esquinas:
        dibujar_marcador(ax, id_m, x, y, 25.0, etq)

    # Marcadores extra de 20 mm (IDs 4..7) por si el borde rojo de la bandeja es mas angosto
    ax.text(15, 215, "2) VERSION COMPACTA PARA BORDE ANGOSTO DE BANDEJA (20 x 20 mm, IDs 0..3):",
            ha="left", va="top", fontsize=8.5, fontweight="bold")
    for idx, (id_m, _, _, etq) in enumerate(esquinas):
        dibujar_marcador(ax, id_m, 22 + idx * 46, 183, 20.0, etq)

    # --- SECCION 3: 4 Marcadores de 40 mm para seguimiento 3D del Dron / Carrito ---
    ax.text(15, 166, "3) SEGUIMIENTO 3D DEL DRON / ALTURA E INCLINACION (40 x 40 mm, IDs 10..13):",
            ha="left", va="top", fontsize=8.5, fontweight="bold")
    dron_mks = [
        (10, 16, 114, "Dron Frente (M1)"),
        (11, 65, 114, "Dron Derecha (M2)"),
        (12, 114, 114, "Dron Atras (M3)"),
        (13, 163, 114, "Base Fija Ref"),
    ]
    for id_m, x, y, etq in dron_mks:
        dibujar_marcador(ax, id_m, x, y, 36.0, etq)

    # --- SECCION 4: Plantilla de centro y puntos de calibracion para la camara OV2640 ---
    ax.text(15, 96, "4) PLANTILLA DE CALIBRACION PARA LA CAMARA DE LA BANDEJA (comandos 'centro' y 'esquinas'):",
            ha="left", va="top", fontsize=8.5, fontweight="bold")
    ax.text(15, 90,
            "   Nota: La ESP32-S3-CAM detecta la bola directo a 50 Hz por contraste/color (no necesita ArUco pegado en la bola).\n"
            "   Usa estos discos blancos/negros o las 4 esquinas de la zona negra para calibrar 'centro' y 'esquinas'.",
            ha="left", va="top", fontsize=7.5)

    # Discos de prueba de diametro 20 mm y 30 mm
    for cx, r, col, txt in [(35, 10, "black", "Disco oscuro Ø20mm\n(modo 1)"),
                            (75, 12.5, "black", "Disco oscuro Ø25mm\n(modo 1)"),
                            (120, 10, "white", "Disco claro Ø20mm\n(modo 0)"),
                            (165, 12.5, "white", "Disco claro Ø25mm\n(modo 0)")]:
        if col == "white":
            ax.add_patch(Rectangle((cx - 18, 42), 36, 36, facecolor="black", edgecolor="black"))
            ax.add_patch(Circle((cx, 60), r, facecolor="white", edgecolor="white"))
        else:
            ax.add_patch(Rectangle((cx - 18, 42), 36, 36, facecolor="white", edgecolor="#666666", linestyle="--"))
            ax.add_patch(Circle((cx, 60), r, facecolor="black", edgecolor="black"))
        ax.text(cx, 38, txt, ha="center", va="top", fontsize=7)

    # --- Regla de 100 mm al pie para verificar impresion 1:1 ---
    x0, y0 = 18, 14
    ax.plot([x0, x0 + 100], [y0, y0], color="black", linewidth=1.5)
    for mm in range(0, 101, 5):
        h = 4.0 if mm % 10 == 0 else 2.2
        ax.plot([x0 + mm, x0 + mm], [y0, y0 + h], color="black", linewidth=0.9)
        if mm % 10 == 0:
            ax.text(x0 + mm, y0 - 1.5, f"{mm}", ha="center", va="top", fontsize=7)
    ax.text(x0 + 104, y0 + 1.5, "<-- Regla de 100 mm (verificar escala 1:1)",
            ha="left", va="center", fontsize=7.5, fontweight="bold")

    fig.savefig(RUTA_PDF, format="pdf")
    fig.savefig(RUTA_PNG, format="png", dpi=200)
    plt.close(fig)
    print(f"PDF generado: {RUTA_PDF}")
    print(f"PNG generado: {RUTA_PNG}")


if __name__ == "__main__":
    main()
