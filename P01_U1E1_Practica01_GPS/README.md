Práctica 1
----------------------------------------------------
codigo de arduino
----------------------------------------------------
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Pines para el GPS
static const int RXPin = 4; // GPS TX
static const int TXPin = 3; // GPS RX
static const uint32_t GPSBaud = 9600;

// Objetos
SoftwareSerial gpsSerial(RXPin, TXPin);
TinyGPSPlus gps;

bool encabezado = false;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);

  // Encabezado de la tabla
  Serial.println("latitud,longitud");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());

    if (gps.location.isUpdated()) {
      Serial.print(gps.location.lat(), 6);
      Serial.print(",");
      Serial.println(gps.location.lng(), 6);

      delay(1000); // 1 punto por segundo
    }
  }
}

----------------------------------------------------
codigo de errores de matlab
---------------------------------------------------
clc; 
clear; 
close all;

escenarios = ["interior","aire_libre","entre_edificios","caminata_200m"];
R = 6371000;   % Radio de la Tierra (m)

for k = 1:length(escenarios)

    fprintf('\n=== Escenario: %s ===\n', escenarios(k));

    %% --- Cargar datos del modulo (CSV) ---
    gps = readtable(fullfile(escenarios(k), "gps_modulo.csv"));

    % Ajusta si tus columnas tienen otros nombres
    latGPS = gps.latitud;
    lonGPS = gps.longitud;
    altGPS = gps.altitud_m;

    %% --- Cargar datos del celular (MAT) ---
    load(fullfile(escenarios(k), "gps_celular.mat"));

    % Variables tipicas de MATLAB Mobile
    latCel = Position.latitude;
    lonCel = Position.longitude;
    altCel = Position.altitude;

    %% --- Igualar longitudes ---
    n = min([length(latGPS), length(latCel)]);
    latGPS = latGPS(1:n); lonGPS = lonGPS(1:n);
    latCel = latCel(1:n); lonCel = lonCel(1:n);

    %% --- Trayectoria ---
    figure
    plot(lonCel, latCel, 'b.-')
    hold on
    plot(lonGPS, latGPS, 'r.-')
    title(['Trayectoria - ' char(escenarios(k))])
    legend('Celular','Modulo GPS')
    xlabel('Longitud')
    ylabel('Latitud')
    grid on

    %% --- Error (Haversine) ---
    lat1 = deg2rad(latCel);
    lon1 = deg2rad(lonCel);
    lat2 = deg2rad(latGPS);
    lon2 = deg2rad(lonGPS);

    a = sin((lat2-lat1)/2).^2 + cos(lat1).*cos(lat2).*sin((lon2-lon1)/2).^2;
    c = 2*atan2(sqrt(a), sqrt(1-a));
    error_m = R * c;

    fprintf('Error promedio: %.2f m\n', mean(error_m));
    fprintf('Desviacion estandar: %.2f m\n', std(error_m));

    %% --- Grafica del error ---
    figure
    plot(error_m)
    title(['Error - ' char(escenarios(k))])
    xlabel('Muestras')
    ylabel('Error (m)')
    grid on
end

--------------------------------------------------------------------------------------------
codigo de trayectorias en matlab
--------------------------------------------------------------------------------------------
%% --- GRAFICADOR GENÉRICO DE GPS ---
clear; clc; close all;

% 1. ABRIR VENTANA DE SELECCIÓN DE ARCHIVO
disp('Por favor, selecciona tu archivo de Excel o CSV...');
[nombreArchivo, rutaArchivo] = uigetfile({'.xlsx;.xls;.csv', 'Archivos de Datos (.xlsx, *.csv)'}, ...
                                         'Selecciona tu archivo de coordenadas');

if isequal(nombreArchivo, 0)
    disp('Cancelado por el usuario.');
    return;
end

archivoCompleto = fullfile(rutaArchivo, nombreArchivo);

% 2. LEER DATOS
try
    % readtable es inteligente y detecta si es Excel o CSV
    tablaDatos = readtable(archivoCompleto); 
    
    % Convertir a matriz para facilitar el manejo (por si hay encabezados)
    % Si tu archivo TIENE encabezados, MATLAB los usará como nombres.
    % Si NO TIENE encabezados, usará la primera fila como datos.
    datos = table2array(tablaDatos);
    
    % --- CONFIGURACIÓN DE COLUMNAS ---
    % IMPORTANTE: Ajusta estos números si tus columnas están en otro orden.
    % Por lo general: 1 = Latitud, 2 = Longitud, 3 = Altitud
    columna_latitud = 1; 
    columna_longitud = 2;
    
    % Extraer coordenadas
    lat = datos(:, columna_latitud);
    lon = datos(:, columna_longitud);
    
    % Filtrar posibles ceros o errores (común en GPS al iniciar)
    idx_validos = (lat ~= 0) & (lon ~= 0);
    lat = lat(idx_validos);
    lon = lon(idx_validos);

    % 3. GRAFICAR
    figure('Name', ['Trayectoria: ' nombreArchivo], 'NumberTitle', 'off');
    plot(lon, lat, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 10);
    
    % Formato del gráfico
    grid on;
    axis equal; % VITAL para que el mapa no se vea deformado
    
    xlabel('Longitud (Eje X)');
    ylabel('Latitud (Eje Y)');
    title(['Mapa de Trayectoria GPS - ' nombreArchivo], 'Interpreter', 'none');
    
    % Añadir un punto de inicio (círculo verde) y fin (cuadrado rojo)
    hold on;
    plot(lon(1), lat(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Inicio');
    plot(lon(end), lat(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Fin');
    legend('Trayectoria', 'Inicio', 'Fin');
    hold off;

    disp('Gráfica generada exitosamente.');

catch ME
    errordlg(['Hubo un error al leer el archivo: ' ME.message], 'Error de Lectura');
end





