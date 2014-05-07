//headers for LCD functions 
//Peter Backlund 2013/03/01

void clear_display(void);
void cursor_home(void);
void home_line2(void);      
void char2lcd(char a_char);
void string2lcd(char *lcd_str);
void lcd_init(void);
void int2lcd(uint8_t val);
void int162lcd(uint16_t val);
void long2lcd(uint32_t val);
void double2lcd(double num);
void volt2lcd(uint16_t val);
