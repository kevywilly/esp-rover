//
// Created by Kevin Williams on 9/17/22.
//

#ifndef ESPROVER_DOCROOT_H
#define ESPROVER_DOCROOT_H
static const char * docroot = "<!DOCTYPE html>\n"
                              "<html lang='en'>\n"
                              "<head>\n"
                              "    <meta charset='UTF-8'>\n"
                              "    <title>Title</title>\n"
                              "    <script type='text/javascript' src='https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.1/nipplejs.min.js'></script>\n"
                              "    <script type='text/javascript' src='https://cdnjs.cloudflare.com/ajax/libs/fetch/3.6.2/fetch.min.js'></script>\n"
                              "    <script type='text/javascript'>\n"
                              "        let movement_counter = 0;\n"
                              "        let _heading = 0;\n"
                              "        let _power = 0;\n"
                              "        function drive(heading, power) {\n"
                              "            document.getElementById('results').innerText = `h: ${heading}, p: ${power} %`;\n"
                              "            if((heading === _heading && power === _power)) {\n"
                              "                return;\n"
                              "            }\n"
                              "            if(!(_heading === 0 && _power === 0) && (Math.abs(heading - _heading) < 5) && (Math.abs(power - _power) < 10)) {\n"
                              "                return;\n"
                              "            }\n"
                              "\n"
                              "            (async () => {\n"
                              "                // POST request using fetch with async/await\n"
                              "                const requestOptions = {\n"
                              "                    method: 'POST',\n"
                              "                    headers: { 'Content-Type': 'application/json' },\n"
                              "                    body: JSON.stringify({ cmd: 'drive', heading: heading, power: power })\n"
                              "                };\n"
                              "                const response = await fetch('/api', requestOptions);\n"
                              "\n"
                              "            })();\n"
                              "            movement_counter = 0;\n"
                              "            _heading = heading;\n"
                              "            _power = power;\n"
                              "        }\n"
                              "    </script>\n"
                              "    <style>\n"
                              "        html {font-family:Helvetica;display:inline-block;margin:0px auto;text-align:center;font-size:1.5rem;}\n"
                              "        .container {width: 100%;margin: auto;}\n"
                              "        @media screen and (min-width:800px) {.container{width:50%;}  }\n"
                              "        h1 {color:#0F3376;padding:2vh;}\n"
                              "        p {font-size:1.5rem;}\n"
                              "    </style>\n"
                              "\n"
                              "</head>\n"
                              "<body>\n"
                              "<h1>ESP Rover</h1>\n"
                              "<div class='container'>\n"
                              "    <div id='joystick' ></div>\n"
                              "    <div id='results' ></div>\n"
                              "</div>\n"
                              "<script>\n"
                              "    let joy = nipplejs.create({\n"
                              "        zone: document.getElementById('joystick'),\n"
                              "        mode: 'static',\n"
                              "        position: {left: '50%', top: '50%'},\n"
                              "        color: 'blue',\n"
                              "        size: 400\n"
                              "    });\n"
                              "    joy.on('move', function (evt, data) {\n"
                              "        let heading = (data.angle.degree > 90 ? 450 - data.angle.degree : 90 - data.angle.degree);\n"
                              "        let power = (data.distance / 200.0);\n"
                              "        drive(heading, power);\n"
                              "    });\n"
                              "\n"
                              "    joy.on('end', function (evt, data) {\n"
                              "        drive(0,0)\n"
                              "    });\n"
                              "</script>\n"
                              "</body>\n"
                              "</html>";
#endif //ESPROVER_DOCROOT_H