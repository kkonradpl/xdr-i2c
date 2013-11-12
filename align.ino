void align(uint32_t freq)
{
    // alignment of the antenna circuit
    // these values are individual for each tuner!

    if(freq>=107300)
        DAA = 52;
    else if(freq>=104500)
        DAA = 53;
    else if(freq>=100400)
        DAA = 54;
    else if(freq>=97500)
        DAA = 55;
    else if(freq>=95600)
        DAA = 56;
    else if(freq>=92900)
        DAA = 57;
    else if(freq>=90500)
        DAA = 58;
    else if(freq>=87800)
        DAA = 59;
    else if(freq>=87000)
        DAA = 60;
    else if(freq>=85000)
        DAA = 61;
    else if(freq>=84000)
        DAA = 62;
    else if(freq>=83000)
        DAA = 63;
    else if(freq>=78000)
        DAA = 64;
    else if(freq>=76000)
        DAA = 65;
    else if(freq>=74000)
        DAA = 66;
    else if(freq>=73000)
        DAA = 67;
    else if(freq>=70000)
        DAA = 69;
    else
        DAA = 70;
}

