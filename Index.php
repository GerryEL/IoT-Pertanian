<?php
date_default_timezone_set('Asia/Jakarta');

$host = "localhost";
$username = "root";
$password = "";
$dbname = "iot pertanian monitoring";

try {
    $conn = new PDO("mysql:host=$host;dbname=$dbname", $username, $password);
    $conn->setAttribute(PDO::ATTR_ERRMODE, PDO::ERRMODE_EXCEPTION);
    
    $conn->exec("CREATE TABLE IF NOT EXISTS `sensor data` (
        `id` INT AUTO_INCREMENT PRIMARY KEY,
        `Temperature` FLOAT NOT NULL,
        `Humidity` FLOAT NOT NULL,
        `LdrValue` INT NOT NULL,
        `RainValue` INT NOT NULL,
        `AirQualityPPM` FLOAT NOT NULL,
        `SoilMoisture` INT NOT NULL,
        `Time` TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    )");

} catch(PDOException $e) {
    die("Connection failed: " . $e->getMessage());
}

// Handle data insertion from form
if($_SERVER['REQUEST_METHOD'] === 'POST') {
    try {
        $sql = "INSERT INTO `sensor data` (
            Temperature, Humidity, LdrValue, RainValue, AirQualityPPM, SoilMoisture
        ) VALUES (?, ?, ?, ?, ?, ?)";
        
        $stmt = $conn->prepare($sql);
        $stmt->execute([
            $_POST['temp'],
            $_POST['humidity'],
            $_POST['ldr'],
            $_POST['rain'],
            $_POST['air_quality'],
            $_POST['soil_moisture']
        ]);
        
        // Redirect to monitoring view
        header("Location: ".$_SERVER['PHP_SELF']."?page=monitor");
        exit();
        
    } catch(PDOException $e) {
        die("Error: " . $e->getMessage());
    }
}

// Handle data fetch for charts
if(isset($_GET['action']) && $_GET['action'] === 'get_data') {
    try {
        $stmt = $conn->query("SELECT 
            Temperature,
            Humidity,
            LdrValue,
            RainValue,
            AirQualityPPM,
            SoilMoisture,
            UNIX_TIMESTAMP(Time)*1000 as timestamp
            FROM `sensor data` ORDER BY Time ASC");
        
        header('Content-Type: application/json');
        echo json_encode($stmt->fetchAll(PDO::FETCH_ASSOC));
        exit;
    } catch(PDOException $e) {
        echo json_encode(['error' => $e->getMessage()]);
        exit;
    }
}
?>
<!DOCTYPE html>
<html>
<head>
    <title>Agriculture Monitoring System</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; max-width: 1000px; margin: 0 auto; padding: 20px; }
        .form-container, .chart-container { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 8px; }
        canvas { width: 100% !important; height: 300px !important; }
        .form-group { margin-bottom: 15px; }
        label { display: block; margin-bottom: 5px; }
        input { width: 100%; padding: 8px; }
        button { padding: 10px 20px; background: #4CAF50; color: white; border: none; cursor: pointer; }
        .nav { display: flex; justify-content: space-between; margin-bottom: 20px; }
        .nav button { background: #333; }
        .hidden { display: none; }
    </style>
</head>
<body>
    <div class="nav">
        <button onclick="showPage('form')">Data Entry</button>
        <button onclick="showPage('monitor')">Real-time Monitoring</button>
    </div>

    <div id="formPage" class="form-container">
        <h2>Environmental Data Entry</h2>
        <form method="POST">
            <div class="form-group">
                <label>Temperature (°C):</label>
                <input type="number" name="temp" step="0.1" required>
            </div>

            <div class="form-group">
                <label>Humidity (%):</label>
                <input type="number" name="humidity" step="0.1" required>
            </div>

            <div class="form-group">
                <label>Light Value (LDR):</label>
                <input type="number" name="ldr" min="0" max="1023" required>
            </div>

            <div class="form-group">
                <label>Rain Value:</label>
                <input type="number" name="rain" min="0" max="1023" required>
            </div>

            <div class="form-group">
                <label>Air Quality (PPM):</label>
                <input type="number" name="air_quality" min="0" required>
            </div>

            <div class="form-group">
                <label>Soil Moisture (%):</label>
                <input type="number" name="soil_moisture" step="0.1" required>
            </div>

            <button type="submit">Submit Data</button>
        </form>
    </div>

    <div id="monitorPage" class="hidden">
        <h2>Real-time Sensor Monitoring</h2>
        
        <div class="chart-container">
            <canvas id="temperatureChart"></canvas>
        </div>
        <div class="chart-container">
            <canvas id="humidityChart"></canvas>
        </div>
        <div class="chart-container">
            <canvas id="ldrChart"></canvas>
        </div>
        <div class="chart-container">
            <canvas id="rainChart"></canvas>
        </div>
        <div class="chart-container">
            <canvas id="airQualityChart"></canvas>
        </div>
        <div class="chart-container">
            <canvas id="soilMoistureChart"></canvas>
        </div>
    </div>

    <script>
    // Navigation control
    function showPage(pageId) {
        document.getElementById('formPage').classList.toggle('hidden', pageId !== 'form');
        document.getElementById('monitorPage').classList.toggle('hidden', pageId !== 'monitor');
        
        if(pageId === 'monitor') {
            initCharts();
            startChartUpdates();
        }
    }

    // Initialize charts on first load
    let charts = {};
    function initCharts() {
        if(Object.keys(charts).length === 0) {
            charts = {
                temperature: createChart(document.getElementById('temperatureChart'), 'Temperature (°C)', '#ff6384'),
                humidity: createChart(document.getElementById('humidityChart'), 'Humidity (%)', '#36a2eb'),
                ldr: createChart(document.getElementById('ldrChart'), 'Light Value (LDR)', '#ffce56'),
                rain: createChart(document.getElementById('rainChart'), 'Rain Value', '#4bc0c0'),
                airQuality: createChart(document.getElementById('airQualityChart'), 'Air Quality (PPM)', '#9966ff'),
                soilMoisture: createChart(document.getElementById('soilMoistureChart'), 'Soil Moisture (%)', '#ff9f40')
            };
            updateCharts();
        }
    }

    // Create chart instances
    function createChart(ctx, label, color) {
        return new Chart(ctx, {
            type: 'line',
            data: { datasets: [{
                label: label,
                data: [],
                borderColor: color,
                tension: 0.1,
                borderWidth: 2,
                fill: false
            }]},
            options: {
                responsive: true,
                scales: {
                    x: {
                        type: 'time',
                        time: {
                            unit: 'minute',
                            tooltipFormat: 'MMM dd, HH:mm',
                            displayFormats: { minute: 'HH:mm' }
                        },
                        title: { display: true, text: 'Time' }
                    },
                    y: { 
                        beginAtZero: true,
                        title: { display: true, text: 'Value' }
                    }
                }
            }
        });
    }
    
    // Update charts with data
    async function updateCharts() {
        try {
            const response = await fetch('?action=get_data');
            const data = await response.json();
            
            if (data.length === 0) return;
            
            charts.temperature.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.Temperature}));
            charts.humidity.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.Humidity}));
            charts.ldr.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.LdrValue}));
            charts.rain.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.RainValue}));
            charts.airQuality.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.AirQualityPPM}));
            charts.soilMoisture.data.datasets[0].data = data.map(d => ({x: d.timestamp, y: d.SoilMoisture}));
            
            Object.values(charts).forEach(chart => chart.update());
        } catch (error) {
            console.error('Error updating charts:', error);
        }
    }
    
    // Start periodic updates
    let updateInterval;
    function startChartUpdates() {
        if(updateInterval) clearInterval(updateInterval);
        updateInterval = setInterval(updateCharts, 5000);
    }

    // Initial page setup
    document.addEventListener('DOMContentLoaded', () => {
        <?php if(isset($_GET['page']) && $_GET['page'] === 'monitor'): ?>
            showPage('monitor');
        <?php else: ?>
            showPage('form');
        <?php endif; ?>
    });
    </script>
</body>
</html>