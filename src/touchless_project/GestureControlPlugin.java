import ij.*;
import ij.gui.*;
import ij.process.*;
import ij.plugin.PlugIn;

import java.net.ServerSocket;
import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;

import java.lang.reflect.Method;

public class GestureControlPlugin implements PlugIn {

    private boolean running = true;

    @Override
    public void run(String arg) {
        IJ.log("GestureControlPlugin avviato.");
        try (ServerSocket server = new ServerSocket(50007)) {
            IJ.log("In ascolto su porta 50007...");
            Socket client = server.accept();
            IJ.log("Python connesso!");

            BufferedReader in = new BufferedReader(
                new InputStreamReader(client.getInputStream())
            );

            String line;
            while ((line = in.readLine()) != null && running) {
                processCommand(line);
            }

        } catch (Exception e) {
            IJ.log("Errore: " + e.getMessage());
        }
    }

    private void processCommand(String cmd) {
        String[] parts = cmd.trim().split("\\s+");
        if (parts.length == 0) return;

        ImagePlus imp = WindowManager.getCurrentImage();
        if (imp == null) return;

        ImageCanvas canvas = imp.getCanvas();
        if (canvas == null) return;

        String op = parts[0];

        try {

            switch (op) {

                // ===================================
                // SHIFT_D dx dy  (delta pan)
                // ===================================
                case "SHIFT_D":
                    if (parts.length == 3) {
                        int dx = (int)Math.round(Double.parseDouble(parts[1]));
                        int dy = (int)Math.round(Double.parseDouble(parts[2]));
                        if (dx == 0 && dy == 0) return;

                        Method scrollMethod =
                            ImageCanvas.class.getDeclaredMethod("scroll", int.class, int.class);
                        scrollMethod.setAccessible(true);
                        scrollMethod.invoke(canvas, dx, dy);
                    }
                    break;

                // ===================================
                // ZOOM_D factor  (delta zoom moltiplicativo)
                // ===================================
                case "ZOOM_D":
                    if (parts.length == 2) {
                        double factor = Double.parseDouble(parts[1]);
                        if (Math.abs(factor - 1.0) < 1e-3) return;

                        double current = canvas.getMagnification();
                        double newMag = current * factor;

                        if (newMag < 0.05) newMag = 0.05;
                        if (newMag > 20.0) newMag = 20.0;

                        canvas.setMagnification(newMag);
                        canvas.repaint();
                    }
                    break;

                // ===================================
                // ROT_D ddeg  (delta rotazione in gradi)
                // ===================================
                case "ROT_D":
                    if (parts.length == 2) {
                        double ddeg = Double.parseDouble(parts[1]);
                        if (Math.abs(ddeg) < 0.5) return;

                        ImageProcessor ip = imp.getProcessor();
                        ip.setInterpolationMethod(ImageProcessor.BILINEAR);
                        ip.rotate(ddeg);
                        imp.updateAndDraw();
                    }
                    break;

                // ===================================
                // SLICE_D steps  (delta numero slice)
                // steps > 0 -> slice successiva
                // steps < 0 -> slice precedente
                // ===================================
                case "SLICE_D":
                    if (parts.length == 2) {
                        int steps = (int)Math.round(Double.parseDouble(parts[1]));
                        if (steps == 0) return;

                        int nSlices = imp.getStackSize();
                        if (nSlices <= 1) return;

                        int current = imp.getCurrentSlice();
                        int target = current + steps;

                        if (target < 1) target = 1;
                        if (target > nSlices) target = nSlices;

                        if (target != current) {
                            imp.setSlice(target);
                            imp.updateAndDraw();
                        }
                    }
                    break;
            }

        } catch (Exception e) {
            IJ.log("Errore reflection: " + e.getMessage());
        }
    }
}
