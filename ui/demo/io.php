<?php

//die();

// should add if (isset($_GET['act']))
if ($_GET['act']=='reset') file_put_contents('data.json','[{"x":0,"y":0}]');

if ($_GET['act']=='add')  {

    //if ((file_get_contents('data.json')=='{"time":[],"voltage":[]}')&&($_GET['reset']==0))
    //    {echo 'reset';die();}

    $json=json_decode(file_get_contents('data.json'),true);
    //echo $json[3]['x'];
    //var_dump($json);
    $lengthOfObj =count($json);
    //echo $lengthOfObj;
    $json[$lengthOfObj]['x']=$_GET['x'];  // should add quotes
    $json[$lengthOfObj]['y']=$_GET['y'];
    file_put_contents('data.json',json_encode($json));
}

if ($_GET['act']=='release') file_put_contents('command.json','{"command": 1}');
if ($_GET['act']=='retract') file_put_contents('command.json','{"command": 0}');


if ($_GET['q']=='command') {
    $json=json_decode(file_get_contents('command.json'),true);
    echo $json[command];
}






?>