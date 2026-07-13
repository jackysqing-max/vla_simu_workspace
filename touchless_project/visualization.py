import nibabel as nib
import numpy as np
import pyvista as pv

# === PARAMETRI ===
nifti_path = "t1_150116AR_20150115.nii"   # <-- il tuo file NIfTI

# === CARICAMENTO NIFTI ===
img = nib.load(nifti_path)
data = img.get_fdata().astype(np.float32)

# Se è 4D (fMRI), prendo il primo volume
if data.ndim == 4:
    data = data[..., 0]

# === OPZIONALE: DOWNSAMPLING PER VELOCIZZARE ===
# Se è troppo lento, attiva la riga sotto (fattore 2)
# data = data[::2, ::2, ::2]

# Nessun aumento di contrasto: uso i valori così come sono

# === CREAZIONE OGGETTO PYVISTA ===
volume = pv.wrap(data)

plotter = pv.Plotter()

plotter.add_volume(
    volume,
    cmap="gray",
    opacity="sigmoid",   # curva di opacità standard (nessuna modifica ai dati)
    shade=False,         # spesso più veloce e meno rumoroso
)

plotter.add_axes()
plotter.show_grid()

plotter.show(title="MRI 3D (senza aumento di contrasto)")
