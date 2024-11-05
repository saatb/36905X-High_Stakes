#include "screen/selector.h"
#include "liblvgl/core/lv_event.h"
#include "main.h"
#include "robot/auton.h"

using namespace Robot;

Autonomous::routine autonSelectorScreen::lastAuton;

autonSelectorScreen::autonSelectorScreen(){
// hi!
};

void autonSelectorScreen::autonUiUpdate(lv_event_t *e){
    lv_obj_t *tab1			 = lv_event_get_current_target(e);
	lv_obj_t *event_obj		 = lv_event_get_target(e);
	lv_obj_t *autonLabel	 = lv_obj_get_child(tab1, 2);
	lv_obj_t *allianceSwitch = lv_obj_get_child(tab1, 3);
	lv_obj_t *skillSwitch	 = lv_obj_get_child(tab1, 4);
	lv_obj_t *auton_dd		 = lv_obj_get_child(tab1, 5);

    if (event_obj == auton_dd) {
        //skills routine is 0 so index should start at 1
		int	 currentAutonIndex = lv_dropdown_get_selected(auton_dd) + 1;
		bool currentAlliance   = lv_obj_has_state(allianceSwitch, LV_STATE_CHECKED);
		int	 autonNum		   = currentAlliance ? currentAutonIndex * -1 : currentAutonIndex;
		Autonomous::autonSwitcher(autonNum);
    }
    else if (event_obj == allianceSwitch) {
		if (lv_obj_has_state(allianceSwitch, LV_STATE_CHECKED)) {
			lv_dropdown_clear_options(auton_dd);
			lv_dropdown_set_options(auton_dd, autonSelectorScreen::blueAutons);
			lv_obj_set_style_border_color(auton_dd, lv_color_hex(0x0077c8), 0);
		} 
        else {
			lv_dropdown_clear_options(auton_dd);
			lv_dropdown_set_options(auton_dd, autonSelectorScreen::redAutons);
			lv_obj_set_style_border_color(auton_dd, lv_color_hex(0xd22730), 0);
		}
        //switches color and accounts for option reset
		Autonomous::autonSwitcher(Autonomous::auton > 0 ? Autonomous::blueLeft : Autonomous::redLeft);
	} else {
		if (lv_obj_has_state(skillSwitch, LV_STATE_CHECKED)) {
			// remembers last comp auton
			autonSelectorScreen::lastAuton = Autonomous::auton;

			// changes routine to skills
			Autonomous::autonSwitcher(Autonomous::skills);
			lv_obj_add_state(allianceSwitch, LV_STATE_DISABLED);
			lv_obj_add_state(auton_dd, LV_STATE_DISABLED);
		} else {
			Autonomous::autonSwitcher(autonSelectorScreen::lastAuton);
			lv_obj_clear_state(allianceSwitch, LV_STATE_DISABLED);
			lv_obj_clear_state(auton_dd, LV_STATE_DISABLED);
		}
	}
	lv_label_set_text_fmt(autonLabel, "Current Auton: %s", Autonomous::autonName.c_str());
}

void autonSelectorScreen::selector(){
	//Create a Tab view object
	lv_obj_t *tabview;
	tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 35);

	//make tab
	lv_obj_t *tab1 = lv_tabview_add_tab(tabview, "Autonomous select");

	lv_obj_add_event_cb(tab1, autonUiUpdate, LV_EVENT_VALUE_CHANGED, NULL);
	lv_obj_t *tabButtons = lv_tabview_get_tab_btns(tabview);
	lv_obj_set_style_bg_color(tabButtons, lv_color_hex(0x4d0000), 0);
	lv_obj_set_style_text_font(tabButtons, &lv_font_montserrat_18, 0);

	lv_obj_t *label1	= lv_label_create(tab1);
	lv_obj_t *label2	= lv_label_create(tab1);
	lv_obj_t *autonName = lv_label_create(tab1);
	lv_label_set_text(label1, "Alliance");
	lv_label_set_text(label2, "Enable Skills");
	lv_label_set_text_fmt(autonName, "Current Auton: %s", Autonomous::autonName.c_str());
	lv_obj_align(label1, LV_ALIGN_TOP_LEFT, 0, 10);
	lv_obj_align(label2, LV_ALIGN_LEFT_MID, 0, 0);
	lv_obj_align(autonName, LV_ALIGN_BOTTOM_MID, 0, 0);
	lv_obj_set_style_text_font(label1, &lv_font_montserrat_20, 0);
	lv_obj_set_style_text_font(label2, &lv_font_montserrat_20, 0);
	lv_obj_set_style_text_font(autonName, &lv_font_montserrat_16, 0);

	//alliance color switch
	lv_obj_t *matchSwitch = lv_switch_create(tab1);
	lv_obj_add_flag(matchSwitch, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_obj_set_style_bg_color(matchSwitch, lv_palette_main(LV_PALETTE_RED), LV_STATE_DEFAULT);
	lv_obj_set_size(matchSwitch, lv_pct(21), lv_pct(27));
	lv_obj_set_style_pad_all(matchSwitch, -5, LV_PART_KNOB);
	lv_obj_align(matchSwitch, LV_ALIGN_TOP_MID, 0, 0);

	//skills switch
	lv_obj_t *skillSwitch = lv_switch_create(tab1);
	lv_obj_add_flag(skillSwitch, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_obj_set_size(skillSwitch, lv_pct(21), lv_pct(27));
	lv_obj_set_style_pad_all(skillSwitch, -5, LV_PART_KNOB);
	lv_obj_align(skillSwitch, LV_ALIGN_CENTER, 0, 0);

	//drop down
	lv_obj_t *auton_dd = lv_dropdown_create(tab1);
	lv_obj_add_flag(auton_dd, LV_OBJ_FLAG_EVENT_BUBBLE);
	lv_dropdown_set_options(auton_dd, autonSelectorScreen::redAutons);
	lv_obj_set_style_max_height(auton_dd, 50, 0);
	lv_obj_set_size(auton_dd, lv_pct(35), lv_pct(35));
	lv_obj_set_style_pad_top(auton_dd, 10, 0);
	lv_obj_set_style_pad_bottom(auton_dd, 10, 0);
	lv_obj_set_style_border_width(auton_dd, 4, 0);
	lv_obj_set_style_border_color(auton_dd, lv_color_hex(0xd22730), 0);
	lv_obj_set_style_border_color(auton_dd, lv_color_hex(0x7a7a7a), LV_STATE_DISABLED);
	lv_obj_align(auton_dd, LV_ALIGN_TOP_RIGHT, 0, 0);
} 