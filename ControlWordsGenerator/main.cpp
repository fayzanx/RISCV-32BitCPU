// Example program
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <bitset>
#include <fstream>

using namespace std;
char lochexa[4];
char wordHex[4];
void generateLocCodes(int b30, int opcode, int f3, int BrLt, int BrEq, string commandname){
    string codes[8];
    int flagA = 0, flagB = 0 , flagC = 0;

    if(BrLt==-1)
        flagC = 1;
    if(BrEq==-1)
        flagB = 1;
    if(b30==-1)
        flagA = 1;

    fstream saveTheWord;
    saveTheWord.open("../Commands.txt", ios::app);

    for(int i=0; i<2; i++){

            if(flagA==1)
                b30 = i;

            for(int j=0; j<2; j++){

                    if(flagB==1)
                        BrEq = j;


                    for(int k=0; k<2; k++){

                        if(flagC==1)
                            BrLt = k;

                        ostringstream wordlocation;
                        wordlocation
                        << bitset<1>(b30).to_string()
                        << bitset<3>(f3).to_string()
                        << bitset<5>(opcode).to_string()
                        << bitset<1>(BrEq).to_string()
                        << bitset<1>(BrLt).to_string();

                        string wordlocbin = wordlocation.str();
                        cout << " >> BIN: " << wordlocbin << endl;
                        int wordInt = stoi(wordlocbin, NULL, 2);

                        itoa(wordInt, lochexa, 16);

                        cout << " >> HEX: " << lochexa << endl;

    saveTheWord <<
    "\n" << setw(10) << commandname << "\t" << setfill('0') << setw(8) <<  wordHex << "\t" << setw(8) <<  lochexa << setfill(' ');


                        if(flagC!=1)
                            break;
                    }

                    if(flagB!=1)
                        break;
            }

            if(flagA!=1)
                break;
    }

    saveTheWord << endl;
    if(saveTheWord)
        cout << "[...] Saved the command(s) in file." << endl;
}

int main()
{
  int opcodes[] = {0, 4, 6, 8, 12, 14, 24};
  string instype[] = {"L", "I", "I (wide)", "S", "R (add)", "R (wide)", "SB (branch)"};

  int exita, ch;
  do{
    system("cls");
    int PCSel, ImmSel, BrUn, ASel, BSel, ALUSel, MemRW, RegRW, WBSel, BrEq, BrLt, B30, opcode;
    int values[7];

    int automatic = 1;
    cout << ">> AUTO[1/0]? : ";
    cin >> automatic;
    if(automatic == 1){
        int type;
        do{
          for (int i=0; i<7; i++){
            cout << i << ". " << instype[i] << " type \n";
          }
          cout << " [~] Type? :";
          cin >> type;
        }while(type <0 || type>6);

        BrEq = -1;
        BrLt = -1;
        B30  = -1;
        switch(type){
          case 0: //L type
            PCSel = 0;
            ImmSel = 0; // Imm = I
            BrUn = 0;
            ASel = 0;
            BSel = 1;
            ALUSel = 0;
            MemRW = 0;
            RegRW = 1;
            WBSel = 1;
            break;
          case 1: //I Type
            PCSel = 0;
            ImmSel = 0;
            BrUn = 0;
            ASel = 0;
            BSel = 1;
            ALUSel = 0;
            MemRW = 0;
            RegRW = 1;
            WBSel = 0;
            break;
          case 2: //I wide
            PCSel = 0;
            ImmSel = 0;
            BrUn = 0;
            ASel = 0;
            BSel = 1;
            ALUSel = 0;
            MemRW = 0;
            RegRW = 1;
            WBSel = 0;
            break;
          case 3: // S
            PCSel = 0;
            ImmSel = 1; //Imm Type = 1
            BrUn = 0;
            ASel = 0;
            BSel = 1;
            ALUSel = 0;
            MemRW = 1;
            RegRW = 0;
            WBSel = 0;
            break;
          case 4: //R
            PCSel = 0;
            ImmSel = 0;
            BrUn = 0;
            ASel = 0;
            BSel = 0;
            ALUSel = 0;
            MemRW = 0;
            RegRW = 1;
            WBSel = 0;
            B30=0;
            break;
          case 5: // R sub
            PCSel = 0;
            ImmSel = 0;
            BrUn = 0;
            ASel = 0;
            BSel = 0;
            ALUSel = 12;
            MemRW = 0;
            RegRW = 1;
            WBSel = 0;
            B30=0;
            break;
          case 6: //SB branch
            PCSel = 1;
            ImmSel = 2; //ImmSel = B
            BrUn = 0; //change this later
            ASel = 1; //Brach
            BSel = 1;
            ALUSel = 0;
            MemRW = 0;
            RegRW = 0;
            WBSel = 0;
            BrEq = 1;   //to be changed later
            BrLt = 1;
            break;
        }


        opcode = opcodes[type];

    } else {
        cout << "PCSel = "; cin >> PCSel;
        cout << "ImmSel = "; cin >> ImmSel;
        cout << "BrUn = "; cin >> BrUn;
        cout << "ASel = "; cin >> ASel;
        cout << "BSel = "; cin >> BSel;
        cout << "ALUSel = "; cin >>ALUSel;
        cout << "MemRW = "; cin >> MemRW;
        cout << "RegRW = "; cin >> RegRW;
        cout << "WBSel = "; cin >> WBSel;
        cout << "BrLt = "; cin >> BrLt;
        cout << "BrEq = "; cin >> BrEq;
        cout << "OPCODE = "; cin >> opcode;
        cout << "Byte30 = "; cin >> B30;
        cout << endl;
    }

    int test = 15;
    ostringstream outWord;
    outWord << PCSel
    << bitset<3>(ImmSel).to_string()
    << BrUn << ASel << BSel
    << bitset<4>(ALUSel).to_string()
    << MemRW << RegRW
    << bitset<2>(WBSel).to_string();

    cout << " VERIFY: "<< "PCSel = "  << PCSel << ", ImmSel = "  << ImmSel
    << ", BrUn = " << BrUn <<  ", ASel = " << ASel  << ", BSel = "  << endl << BSel
    << ", ALUSel = " << ALUSel << ", MemRW = "  << MemRW << ", RegRW = "
    << RegRW << ", WBSel = " << WBSel << ", BrLt = " << BrLt << ", BrEq = " << BrEq << endl;

    cout << "\n\tCONTROL WORD" << endl;
    string wordBin = outWord.str();
    cout << "\n >> BIN: " << wordBin;

    int wordInt = stoi(wordBin, NULL, 2);

    itoa(wordInt, wordHex, 16);

    cout << "\n >> HEX: " << wordHex << endl;

    int f3 = 0; //, b30 = 0, brlt = 0, breq = 0; //, opcode;
    //opcode = opcodes[type];
    cout << "\n\tLOCATION: " << endl;
    cout << " << OPCODE: " << bitset<5>(opcode).to_string() << endl;
    cout << " >> FUNC3:  "; cin >> f3;
    //cout << " >> BIT30:  "; cin >> b30;
    //cout << " >> BRLT:   "; cin >> brlt;
    //cout << " >> BREQ:   "; cin >> breq;


    string commandname;
    cout << " [~] Give it a name: "; cin >> commandname;

    generateLocCodes(B30, opcode, f3, BrLt, BrEq, commandname);
    /*ostringstream wordlocation;
    wordlocation
    << bitset<1>(b30).to_string()
    << bitset<5>(opcode).to_string()
    << bitset<3>(f3).to_string()
    << bitset<1>(breq).to_string()
    << bitset<1>(brlt).to_string()

    string wordlocbin = wordlocation.str();
    cout << " >> BIN: " << wordlocbin << endl;
    wordInt = stoi(wordlocbin, NULL, 2);
    char lochexa[4];
    itoa(wordInt, lochexa, 16);

    /*if(BrEq == -1 && BrLt == -1)
    cout << ">> HEX: " << lochexa << "[3:0]" endl;
    if(BrEq != -1 && BrLt == -1)
    cout << ">> HEX: " << lochexa << "[1:0]" endl;
    if(BrEq == -1 && BrLt != -1)
    cout << ">> HEX: " << lochexa << "[3|0]" endl;**/


    cout << "\n [~] Exit? [1/0]: ";
    cin >> exita;
  }while(!exita);
}
