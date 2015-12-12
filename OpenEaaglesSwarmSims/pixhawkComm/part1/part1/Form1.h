#pragma once

#include <iostream>
#include <fstream>
#include <string>

# define PI 3.14159265358979323846

namespace part1 {

	using namespace std;
	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace System::IO::Ports;

public ref class Form1 : public System::Windows::Forms::Form
{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			findPorts();
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::ComboBox^  comboBox1;
	private: System::IO::Ports::SerialPort^  serialPort1;
	private: System::Windows::Forms::ComboBox^  comboBox2;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Button^  button4;
	private: System::Windows::Forms::ProgressBar^  progressBar1;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::ProgressBar^  rollProgressBar;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;


	private: System::Windows::Forms::ProgressBar^  pitchProgressBar;
	private: System::Windows::Forms::ProgressBar^  yawProgressBar;




	private: System::ComponentModel::IContainer^  components;
	protected: 

	private:


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
			this->serialPort1 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->comboBox2 = (gcnew System::Windows::Forms::ComboBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->progressBar1 = (gcnew System::Windows::Forms::ProgressBar());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->rollProgressBar = (gcnew System::Windows::Forms::ProgressBar());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->pitchProgressBar = (gcnew System::Windows::Forms::ProgressBar());
			this->yawProgressBar = (gcnew System::Windows::Forms::ProgressBar());
			this->SuspendLayout();
			// 
			// comboBox1
			// 
			this->comboBox1->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBox1->FormattingEnabled = true;
			this->comboBox1->Location = System::Drawing::Point(355, 29);
			this->comboBox1->Name = L"comboBox1";
			this->comboBox1->Size = System::Drawing::Size(121, 21);
			this->comboBox1->TabIndex = 0;
			// 
			// serialPort1
			// 
			this->serialPort1->ReadTimeout = 500;
			this->serialPort1->WriteTimeout = 500;
			// 
			// comboBox2
			// 
			this->comboBox2->DropDownStyle = System::Windows::Forms::ComboBoxStyle::DropDownList;
			this->comboBox2->FormattingEnabled = true;
			this->comboBox2->Items->AddRange(gcnew cli::array< System::Object^  >(2) {L"9600", L"115200"});
			this->comboBox2->Location = System::Drawing::Point(355, 69);
			this->comboBox2->Name = L"comboBox2";
			this->comboBox2->Size = System::Drawing::Size(121, 21);
			this->comboBox2->TabIndex = 1;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(285, 32);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(58, 13);
			this->label1->TabIndex = 2;
			this->label1->Text = L"COM Ports";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(285, 76);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(58, 13);
			this->label2->TabIndex = 3;
			this->label2->Text = L"Baud Rate";
			// 
			// button1
			// 
			this->button1->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button1->Location = System::Drawing::Point(355, 111);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(90, 55);
			this->button1->TabIndex = 4;
			this->button1->Text = L"Connect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// button2
			// 
			this->button2->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button2->Location = System::Drawing::Point(355, 172);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(90, 55);
			this->button2->TabIndex = 5;
			this->button2->Text = L"Disconnect";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// textBox1
			// 
			this->textBox1->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(128)));
			this->textBox1->Enabled = false;
			this->textBox1->Location = System::Drawing::Point(59, 99);
			this->textBox1->Name = L"textBox1";
			this->textBox1->ReadOnly = true;
			this->textBox1->Size = System::Drawing::Size(170, 20);
			this->textBox1->TabIndex = 6;
			this->textBox1->Text = L"Received Here";
			// 
			// textBox2
			// 
			this->textBox2->BackColor = System::Drawing::Color::FromArgb(static_cast<System::Int32>(static_cast<System::Byte>(255)), static_cast<System::Int32>(static_cast<System::Byte>(192)), 
				static_cast<System::Int32>(static_cast<System::Byte>(128)));
			this->textBox2->Location = System::Drawing::Point(59, 190);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(170, 20);
			this->textBox2->TabIndex = 7;
			this->textBox2->Text = L"Enter Here";
			// 
			// button3
			// 
			this->button3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button3->Location = System::Drawing::Point(59, 135);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(75, 40);
			this->button3->TabIndex = 8;
			this->button3->Text = L"Send";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
			// 
			// button4
			// 
			this->button4->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->button4->Location = System::Drawing::Point(148, 135);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(81, 40);
			this->button4->TabIndex = 9;
			this->button4->Text = L"Read";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &Form1::button4_Click);
			// 
			// progressBar1
			// 
			this->progressBar1->Location = System::Drawing::Point(59, 46);
			this->progressBar1->Name = L"progressBar1";
			this->progressBar1->Size = System::Drawing::Size(170, 28);
			this->progressBar1->TabIndex = 10;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point, 
				static_cast<System::Byte>(0)));
			this->label3->Location = System::Drawing::Point(56, 19);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(70, 13);
			this->label3->TabIndex = 11;
			this->label3->Text = L"Port Status";
			// 
			// rollProgressBar
			// 
			this->rollProgressBar->Location = System::Drawing::Point(68, 263);
			this->rollProgressBar->Name = L"rollProgressBar";
			this->rollProgressBar->Size = System::Drawing::Size(523, 23);
			this->rollProgressBar->TabIndex = 12;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(31, 273);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(25, 13);
			this->label4->TabIndex = 13;
			this->label4->Text = L"Roll";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(31, 303);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(31, 13);
			this->label5->TabIndex = 14;
			this->label5->Text = L"Pitch";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(31, 332);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(28, 13);
			this->label6->TabIndex = 15;
			this->label6->Text = L"Yaw";
			// 
			// pitchProgressBar
			// 
			this->pitchProgressBar->Location = System::Drawing::Point(68, 293);
			this->pitchProgressBar->Name = L"pitchProgressBar";
			this->pitchProgressBar->Size = System::Drawing::Size(523, 23);
			this->pitchProgressBar->TabIndex = 18;
			// 
			// yawProgressBar
			// 
			this->yawProgressBar->Location = System::Drawing::Point(68, 322);
			this->yawProgressBar->Name = L"yawProgressBar";
			this->yawProgressBar->Size = System::Drawing::Size(523, 23);
			this->yawProgressBar->TabIndex = 19;
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::Color::Silver;
			this->ClientSize = System::Drawing::Size(612, 381);
			this->Controls->Add(this->yawProgressBar);
			this->Controls->Add(this->pitchProgressBar);
			this->Controls->Add(this->label6);
			this->Controls->Add(this->label5);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->rollProgressBar);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->progressBar1);
			this->Controls->Add(this->button4);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->textBox2);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->comboBox2);
			this->Controls->Add(this->comboBox1);
			this->Name = L"Form1";
			this->Text = L"Form1";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion


	// find available ports
	private: void findPorts(void) {
		// get port names
		array<Object^>^ objectArray = SerialPort::GetPortNames();
		// add string array to combobox
		this->comboBox1->Items->AddRange( objectArray );
	}

	// connect button
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
		this->textBox1->Text=String::Empty;
		if(this->comboBox1->Text==String::Empty || this->comboBox2->Text==String::Empty) {
			this->textBox1->Text="Please Select Port Settings";
		} else {
			try{
				// make sure port isn't open	
				if(!this->serialPort1->IsOpen) {
					this->serialPort1->PortName=this->comboBox1->Text;
					this->serialPort1->BaudRate=Int32::Parse("9600");
					this->textBox2->Text="Enter Message Here";
					//open serial port 
					this->serialPort1->Open();
					this->progressBar1->Value=100;
				} else {
					this->textBox2->Text="Port isn't openned";
				}
			} catch(UnauthorizedAccessException^) {
				this->textBox2->Text="UnauthorizedAccess";
			}
		}
	}

	// close button
	private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
		//close serialPort
		this->serialPort1->Close();
		// update progress bar
		this->progressBar1->Value=0;
		// Enable read button
		this->button4->Enabled = true;
		// Enable the init button
		this->button1->Enabled = true;
	}

	// send button
	private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
		// add sender name
		String^ name = this->serialPort1->PortName;
		// grab text and store in send buffer
		String^ message = this->textBox2->Text;
		// write to serial
		if(this->serialPort1->IsOpen) {
			//this->_serialPort->WriteLine(String::Format("<{0}>: {1}",name,message));
			this->serialPort1->WriteLine(message);
		} else {
			this->textBox2->Text="Port Not Opened";
		}
	}

	//read button
	private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
		// check if port is ready for reading
		if(this->serialPort1->IsOpen){
			// Reset the text in the result label.
			this->textBox1->Text = String::Empty;
			try{	  
				this->textBox1->Text = "Capturing...";
				//std::ofstream myfile;
				//myfile.open("hello.txt", std::ofstream::app);
				//myfile << "Writing some stuff to file...";
				//myfile.close();
				int state = 0;
				//s= 0    1    2    3    4    5    6    7
				int hdr, len, seq, sid, cid, mid, cs1, cs2;
				int dat = 0;
				int pkt[256];
				
				for(int qty = 0; qty < 10000;) {
				
					while(this->serialPort1->BytesToRead == 0) { /* DO NOTHING AND WAIT */ }
				
					// get next byte
					int b = this->serialPort1->ReadByte();
				
					switch(state) {
					case 0:
						if(b==254) {
							hdr = b;
							state++;
						}
						break;
					case 1:
						len = b;
						state++;
						break;
					case 2:
						seq = b;
						state++;
						break;
					case 3:
						sid = b;
						state++;
						break;
					case 4:
						cid = b;
						state++;
						break;
					case 5:
						mid = b;
						state++;
						break;
					case 6:
						if(dat < len) {
							pkt[dat] = b;
							dat++;
						} else {
							cs1 = b;
							dat = 0;
							state++;
						}
						break;
					case 7:
						cs2 = b;
						state = 0;
						qty++;
				
						if(mid == 30) {
							float roll, pitch, yaw;
				
							unsigned char r[] = {pkt[4],pkt[5],pkt[6],pkt[7]};	 // roll
							memcpy(&roll, &r, sizeof(roll));
							unsigned char p[] = {pkt[8],pkt[9],pkt[10],pkt[11]};	 // pitch
							memcpy(&pitch, &p, sizeof(pitch));
							unsigned char y[] = {pkt[12],pkt[13],pkt[14],pkt[15]}; // yaw
							memcpy(&yaw, &y, sizeof(yaw));
				
							
							int roll_int  = (int) (50*roll/PI)+50;
							int pitch_int = (int) (100*pitch/PI)+50;
							int yaw_int   = (int) (50*yaw/PI)+50;
							
							this->rollProgressBar->Value  = roll_int;
							this->pitchProgressBar->Value = pitch_int;
							this->yawProgressBar->Value   = yaw_int;
				
							//b = {pkt[16],pkt[17],pkt[18],pkt[19]}; // rollspeed
							//b = {pkt[20],pkt[21],pkt[22],pkt[23]}; // pitchspeed
							//b = {pkt[24],pkt[25],pkt[26],pkt[27]}; // yawspeed
							//searching = false;
							this->Refresh();
							//while(this->serialPort1->BytesToRead != 0) {
							//	this->serialPort1->ReadByte();
							//}
						}
						break;
					} // end of switch
				
					/*
						// write packet to file
						myfile << hdr << "," << len << "," << seq << "," << sid << "," << cid << "," << mid << "," << cs1 << "," << cs2;
						for(int i = 0; i < len; i++) {
							myfile << "," << pkt[i];
						}
						myfile << "\n";
					*/
				} // end of for loop

// -------------------------------------------------------------------------------------------------------------------------
			} catch(TimeoutException^){
				this->textBox1->Text="Timeout Exception";
			}
			  // Disable the init button
			  // the asynchronous operation is done.
			  this->button1->Enabled = false;
		  }
		  else
			  // give error warning
			 this->textBox1->Text="Port Not Opened";


		 }

}; // End of class Form1
} // End of namespace part1

