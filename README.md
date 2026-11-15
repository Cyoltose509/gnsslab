# GnssLab

A GNSS data-processing lab for Windows: SPP / PPP / RTK positioning, LEO orbit determination, pure-INS navigation, plus observation downloads, result plots and quality analysis.

[Chinese (Simplified)](README_ZH.md)

© 2026 Cyoltose

## Running

Launch `bin/GnssLab.exe` for the GUI, or run `bin/gnss_cli.exe <config.ini>` from the command line. Every key and its values are documented in `gnss.ini` next to the executable.

Switch the interface language from the **Language** menu at the end of the menu bar; on the command line use `--language EN` for a one-off override, or set `[ui] language` in `gnss.ini`.

To build from source: open the project in CLion with the `Windows (Ninja, x64)` preset, and run `git submodule update --init --recursive` after the first checkout. Requires Windows and the Desktop development with C++ workload of VS2022.

## Interface

  * **Task bar**: create SPP / PPP (static, kinematic) / RTK / LEO / INS jobs.
  * **Files tab**: choose observation files, ephemerides and products; static modes take a constant reference truth, kinematic modes attach an SP3 truth file, and INS jobs take the initial position and velocity here.
  * **Parameters tab**: sampling interval, constellations, elevation cutoff, correction models, troposphere mapping, filter direction and so on; the fields switch with the mode.
  * **Results tab**: coordinates (float / fixed), ECEF velocity, its norm and σ, reference and deviations, DOP and solution status; the satellite table shows pseudorange and phase residuals epoch by epoch; plots and satellite visibility follow the same time axis. INS results switch between the free solution and the ZUPT solution.
  * **Tools**: time / coordinate conversion, least squares, IMU calibration and alignment, INS algorithm verification.
  * **Data**: six tabs — daily precise products / observations / LEO satellites / reference truth (station SINEX, LEO precise SP3 orbits) / auxiliary data / local files; enter Earthdata credentials on the same page.
  * **Help → About**: shows this manual in Chinese or English, following the interface language.

## Export

  * **Formats**: SPP / PPP / RTK write CSV, POS (the plain-text format used by RTKLIB `rtkpost`), SP3-c and KML (a track line readable by Google Earth, QGIS and similar); LEO adds orbital elements; INS writes CSV, track, KML and SBET (a binary track of 17 doubles per record).
  * **Sampling**: as solved / every N seconds / every N epochs; **Elements** picks CSV columns one by one; **Range** can export a single span of epochs.

## Data sources and citation

Observations, precise orbits and clocks, ionosphere and troposphere products come from **IGS** and its data centers (**NASA CDDIS**, **WHU**), **CODE** (AIUB) and TU Wien; LEO spaceborne observations and precise orbits come from **NASA PO.DAAC**, **UCAR CDAAC** (COSMIC-2), **ESA Swarm**, **GFZ ISDC** and **IDS** (Sentinel-6A/6B).

When publishing results based on these data, follow each provider's citation and acknowledgement rules: IGS requires proper citation and acknowledgement ([Disclaimer and Terms of Use](https://igs.org/about/disclaimer/)); NASA data must be cited in the text and listed in the references, and CDDIS / PO.DAAC dataset pages give a recommended format including the DOI and access date ([Data Use Guidance](https://www.earthdata.nasa.gov/engage/open-data-services-software-policies/data-use-guidance)); ESA requires a statement naming ESA as the data source ([Data Policy](https://earth.esa.int/eogateway/documents/20142/1564626/ESA-Data-Policy-ESA-PB-EO-2010-54.pdf)); see the respective site terms for UCAR CDAAC and GFZ ISDC.

For the products themselves: the CODE series is Dach et al., *CODE final product series for the IGS*, DOI [10.48620/97773](https://doi.org/10.48620/97773) (CC0 1.0); IDS Sentinel-6A / Jason orbits are GSFC std2400, DOI [10.24400/312072/i01-2025.001](https://doi.org/10.24400/312072/i01-2025.001).

These organisation names describe data provenance only and do not imply endorsement of this program. The data are for research and teaching only, must not be used in safety-critical systems, and are used at your own risk.

## Open-source components

Copyright remains with the respective authors; full licence texts are in the matching `thirdparty/` directories:

  * [Dear ImGui](https://github.com/ocornut/imgui) — GUI, MIT License
  * [ImPlot](https://github.com/epezent/implot) — 2D plots, MIT License
  * [ImPlot3D](https://github.com/brenocq/implot3d) — 3D plots, MIT License
  * [imgui_markdown](https://github.com/juliettef/imgui_markdown) — Markdown rendering on the About page, zlib License
  * [Eigen](https://eigen.tuxfamily.org) — matrices and least squares, MPL 2.0 (some files BSD / LGPL)
  * [libcurl](https://curl.se) — FTP / HTTPS transfers, curl License (MIT-like)
  * [libarchive](https://libarchive.org) — archive unpacking (gzip / compress / tar / zip), BSD 2-Clause
  * [zlib](https://zlib.net) — deflate decompression for gzip and zip, a dependency of libarchive, zlib License
